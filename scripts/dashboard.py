#!/usr/bin/env python3
"""
RACECAR Neo web dashboard — real-time ROS2 node and topic monitor.

Serves a single-page dashboard on port 8080 showing node health, topic
publish rates, and the tail of the watchdog log. Stdlib only (http.server,
subprocess, threading); no Flask / Tornado / etc.

Designed to run as racecar-dashboard.service.
"""

from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import logging
import os  # noqa: F401  (kept for potential future use)
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

PORT = 8080
CACHE_TTL = 2.0  # seconds to cache status between requests

# One card per node; mirrors the watchdog's monitored set.
MONITORED = {
    'pwm': {'topic': '/motor', 'label': 'PWM (Maestro)'},
    'throttle': {'topic': '/motor', 'label': 'Throttle (clamping)'},
    'mux': {'topic': '/mux_out', 'label': 'Mux (arbitrator)'},
    'gamepad': {'topic': '/gamepad_drive', 'label': 'Gamepad'},
    'imu': {'topic': '/imu', 'label': 'LSM9DS1 IMU'},
    'lidar': {'topic': '/scan', 'label': 'RPLIDAR'},
    'camera_forward': {'topic': '/camera/forward', 'label': 'BRIO (forward)'},
    'camera_backward': {'topic': '/camera/backward', 'label': 'Arducam (backward)'},
    'edgetpu': {'topic': '/edgetpu/inference', 'label': 'Coral EdgeTPU'},
    'dotmatrix': {'topic': '/dotmatrix/pixels', 'label': 'Dot matrix'},
}

# Topics with publish-rate samples in the table view.
RATE_TOPICS = [
    '/motor',
    '/mux_out',
    '/imu',
    '/scan',
    '/camera/forward',
    '/camera/backward',
    '/edgetpu/inference',
]

log = logging.getLogger('dashboard')

# ---------------------------------------------------------------------------
# Status collection (cached, background-refreshed)
# ---------------------------------------------------------------------------

_status_lock = threading.Lock()
_latest_status: dict = {
    'timestamp': '',
    'nodes': {},
    'node_list': [],
    'topic_list': [],
    'rates': {},
    'system_health': {},
    'watchdog_log': [],
    'log_dir': str(Path.home() / 'logs' / 'latest'),
}
_monitor_running = True

# Refresh slow-changing system diagnostics (RTC, under-voltage) at a longer
# interval to avoid hammering vcgencmd / hwmon every ~3 s.
SYSTEM_HEALTH_REFRESH_SEC = 60.0


def _run(cmd, timeout: int = 5) -> str:
    """Run a command and return stdout, or empty string on failure."""
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return r.stdout.strip() if r.returncode == 0 else ''
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
        return ''


def _get_topic_list():
    out = _run(['ros2', 'topic', 'list'], timeout=10)
    return out.splitlines() if out else []


def _get_node_list():
    out = _run(['ros2', 'node', 'list'], timeout=10)
    return out.splitlines() if out else []


def _measure_hz(topic: str):
    """Measure topic rate over a short window. Returns Hz or None."""
    try:
        r = subprocess.run(
            ['ros2', 'topic', 'hz', topic, '--window', '3'],
            capture_output=True, text=True, timeout=15,
        )
        # ros2 topic hz prints to stdout AND stderr depending on version.
        output = r.stdout + '\n' + r.stderr
        for line in output.splitlines():
            if 'average rate' in line:
                return float(line.split(':')[1].strip())
    except subprocess.TimeoutExpired as e:
        # On timeout, partial output may still contain a rate measurement.
        raw = e.stdout or b''
        if isinstance(raw, bytes):
            raw = raw.decode(errors='replace')
        for line in raw.splitlines():
            if 'average rate' in line:
                try:
                    return float(line.split(':')[1].strip())
                except (ValueError, IndexError):
                    pass
    except (ValueError, IndexError, OSError):
        pass
    return None


def _read_watchdog_tail(n: int = 10):
    """Return the last n lines of the watchdog log."""
    logfile = Path.home() / 'logs' / 'latest' / 'watchdog.log'
    if not logfile.exists():
        return []
    try:
        lines = logfile.read_text().strip().splitlines()
        return lines[-n:]
    except OSError:
        return []


# RTC battery thresholds — same as TestRTC. Below 2.7 V the CR2032 can't
# reliably drive the PCF85063 RTC and the clock resets on next power-off.
RTC_OK_VOLTS = 3.0
RTC_LOW_VOLTS = 2.7


def _read_battery_voltage():
    """Read Pi 5 RTC backup battery voltage. Returns volts or None on failure."""
    import re
    try:
        r = subprocess.run(
            ['vcgencmd', 'pmic_read_adc', 'BATT_V'],
            capture_output=True, text=True, timeout=3,
        )
    except (subprocess.TimeoutExpired, FileNotFoundError, OSError):
        return None
    if r.returncode != 0 or 'BATT_V' not in r.stdout:
        return None
    m = re.search(r'BATT_V\s+volt\(\d+\)=([0-9.]+)V', r.stdout)
    return float(m.group(1)) if m else None


def _read_under_voltage_alarm():
    """Pi 5 PMIC sticky low-voltage alarm. Returns True/False or None if unavailable."""
    for h in Path('/sys/class/hwmon').glob('hwmon*'):
        try:
            if (h / 'name').read_text().strip() == 'rpi_volt':
                alarm = h / 'in0_lcrit_alarm'
                if alarm.exists():
                    return alarm.read_text().strip() == '1'
        except OSError:
            continue
    return None


def _classify_rtc(volts):
    """Map RTC battery voltage to a (status, label) tuple for dashboard rendering."""
    if volts is None:
        return ('dead', 'NO READING')
    if volts >= RTC_OK_VOLTS:
        return ('healthy', f'{volts:.2f} V')
    if volts >= RTC_LOW_VOLTS:
        return ('stale', f'{volts:.2f} V — replace soon')
    return ('dead', f'{volts:.2f} V — REPLACE NOW')


def _collect_system_health():
    """Slow-refresh diagnostics: RTC battery + Pi under-voltage alarm."""
    volts = _read_battery_voltage()
    rtc_status, rtc_label = _classify_rtc(volts)
    uv_alarm = _read_under_voltage_alarm()
    if uv_alarm is None:
        uv_status, uv_label = ('dead', 'UNAVAILABLE')
    elif uv_alarm:
        uv_status, uv_label = ('dead', 'TRIPPED (5V dipped this boot)')
    else:
        uv_status, uv_label = ('healthy', 'OK')
    return {
        'rtc': {'label': 'RTC battery', 'status': rtc_status, 'detail': rtc_label},
        'under_voltage': {
            'label': 'Pi under-voltage alarm',
            'status': uv_status,
            'detail': uv_label,
        },
    }


def _monitor_loop() -> None:
    """Background thread that continuously refreshes the cached status snapshot."""
    global _monitor_running
    last_system_health = 0.0
    system_health = _collect_system_health()
    while _monitor_running:
        try:
            # Phase 1: quick check (topic list + node list).
            topics = _get_topic_list()
            nodes = _get_node_list()

            node_status = {}
            for name, cfg in MONITORED.items():
                present = cfg['topic'] in topics
                node_status[name] = {
                    'label': cfg['label'],
                    'topic': cfg['topic'],
                    'alive': present,
                    'status': 'healthy' if present else 'dead',
                }

            # Phase 2: measure rates in parallel threads to keep refresh < 15 s.
            rate_results: dict = {}
            alive_topics = [t for t in RATE_TOPICS if t in topics]

            def measure(topic):
                rate_results[topic] = _measure_hz(topic)

            threads = []
            for topic in alive_topics:
                t = threading.Thread(target=measure, args=(topic,), daemon=True)
                t.start()
                threads.append(t)
            for t in threads:
                t.join(timeout=10)

            rates = {}
            for topic in RATE_TOPICS:
                if topic in rate_results:
                    hz = rate_results[topic]
                    rates[topic] = {'hz': hz, 'stale': hz is None or hz < 0.5}
                else:
                    rates[topic] = {'hz': None, 'stale': True}

            # Phase 3: slow-refresh system diagnostics (RTC + under-voltage).
            now = time.monotonic()
            if now - last_system_health >= SYSTEM_HEALTH_REFRESH_SEC:
                system_health = _collect_system_health()
                last_system_health = now

            status = {
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                'nodes': node_status,
                'node_list': nodes,
                'topic_list': topics,
                'rates': rates,
                'system_health': system_health,
                'watchdog_log': _read_watchdog_tail(),
                'log_dir': str(Path.home() / 'logs' / 'latest'),
            }

            with _status_lock:
                _latest_status.update(status)

        except Exception:  # noqa: BLE001
            log.exception('Error in monitor loop')

        # Sleep between updates in short increments for clean shutdown.
        for _ in range(30):
            if not _monitor_running:
                break
            time.sleep(0.1)


def get_status() -> dict:
    """Return the most recent status snapshot (non-blocking)."""
    with _status_lock:
        return dict(_latest_status)


# ---------------------------------------------------------------------------
# HTTP handler — HTML lives in scripts/dashboard.html so flake8 stays happy.
# ---------------------------------------------------------------------------

_HTML_PATH = Path(__file__).resolve().parent / 'dashboard.html'


def _load_dashboard_html() -> str:
    """Read the HTML template from disk. Cached at module import time."""
    try:
        return _HTML_PATH.read_text(encoding='utf-8')
    except OSError as exc:
        return f'<!DOCTYPE html><body><pre>dashboard.html unreadable: {exc}</pre>'


DASHBOARD_HTML = _load_dashboard_html()


class DashboardHandler(BaseHTTPRequestHandler):
    """Serve GET / (HTML) and GET /api/status (JSON snapshot)."""

    def do_GET(self):
        if self.path == '/':
            self._serve_html()
        elif self.path == '/api/status':
            self._serve_status()
        else:
            self.send_error(404)

    def _serve_html(self):
        content = DASHBOARD_HTML.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _serve_status(self):
        data = get_status()
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        """Suppress default per-request logging."""
        pass


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    global _monitor_running

    logdir = Path.home() / 'logs' / 'latest'
    handlers = [logging.StreamHandler(sys.stderr)]
    try:
        if logdir.exists():
            fh = logging.FileHandler(logdir / 'dashboard.log')
            handlers.append(fh)
    except OSError:
        pass

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
        handlers=handlers,
    )

    monitor = threading.Thread(target=_monitor_loop, daemon=True)
    monitor.start()
    log.info('Background monitor started')

    server = HTTPServer(('0.0.0.0', PORT), DashboardHandler)
    log.info('Dashboard listening on http://0.0.0.0:%d', PORT)

    def _shutdown(signum, _frame):
        global _monitor_running
        log.info('Received signal %d, shutting down', signum)
        _monitor_running = False
        threading.Thread(target=server.shutdown).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    try:
        server.serve_forever()
    finally:
        _monitor_running = False
        server.server_close()
        monitor.join(timeout=5)
        log.info('Dashboard stopped')


if __name__ == '__main__':
    main()
