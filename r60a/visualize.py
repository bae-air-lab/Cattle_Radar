"""Live web dashboard for the MicRadar R60A.

    python -m r60a.visualize --port auto --baud auto
    python -m r60a.visualize --replay data/session.csv

Serves a self-contained page on http://127.0.0.1:8420 and streams radar state
to it over server-sent events.  Standard library only -- no web framework.
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import threading
import time
import webbrowser
from collections import deque
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Iterator, Sequence

import serial

from .discovery import (
    CANDIDATE_BAUDS,
    BaudProbe,
    DiscoveryError,
    autodetect_baud,
    autodetect_port,
)
from .protocol import (
    ENUM_LABELS,
    MESSAGE_MAP,
    Frame,
    FrameParser,
    UnknownFrame,
    decode,
)
from .record import CsvSink

__all__ = ["main", "VizState", "serve"]

STATIC_DIR = Path(__file__).parent / "static"
DASHBOARD = STATIC_DIR / "dashboard.html"

#: Fields plotted as time series on the dashboard.
CHARTED_FIELDS: tuple[str, ...] = (
    "heart_rate_bpm",
    "breathing_rate_bpm",
    "target_distance_cm",
    "body_movement",
)

HISTORY_POINTS = 240
TRACK_POINTS = 120
LOG_LINES = 60

_RECONNECT_BACKOFF: tuple[float, ...] = (0.5, 1.0, 2.0, 4.0, 8.0, 10.0)


@dataclass
class FieldSnapshot:
    value: float | int | str
    unit: str
    timestamp: float
    count: int


@dataclass
class MessageSnapshot:
    control: int
    command: int
    name: str
    known: bool
    count: int
    last_hex: str
    last_payload: str
    last_seen: float


@dataclass
class VizState:
    """Thread-safe store shared between the radar source and the HTTP handlers."""

    port: str = ""
    baud: int = 0
    mode: str = "live"
    connected: bool = False
    status_detail: str = "starting"
    reconnects: int = 0
    started: float = field(default_factory=time.time)

    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)
    _fields: dict[str, FieldSnapshot] = field(default_factory=dict, repr=False)
    _series: dict[str, deque[tuple[float, float]]] = field(default_factory=dict, repr=False)
    _track: deque[tuple[float, int, int, int]] = field(
        default_factory=lambda: deque(maxlen=TRACK_POINTS), repr=False
    )
    _messages: dict[tuple[int, int], MessageSnapshot] = field(default_factory=dict, repr=False)
    _log: deque[dict[str, Any]] = field(
        default_factory=lambda: deque(maxlen=LOG_LINES), repr=False
    )
    _frame_times: deque[float] = field(
        default_factory=lambda: deque(maxlen=64), repr=False
    )
    _stats: dict[str, int] = field(
        default_factory=lambda: {
            "frames_ok": 0,
            "bad_checksum": 0,
            "unknown": 0,
            "dropped": 0,
            "readings": 0,
        },
        repr=False,
    )

    # -- writer side -------------------------------------------------------

    def set_connection(self, connected: bool, detail: str) -> None:
        with self._lock:
            self.connected = connected
            self.status_detail = detail

    def note_reconnect(self) -> None:
        with self._lock:
            self.reconnects += 1

    def sync_parser_stats(self, parser: FrameParser) -> None:
        with self._lock:
            self._stats["frames_ok"] = parser.stats.frames_ok
            self._stats["bad_checksum"] = parser.stats.frames_bad_checksum
            self._stats["dropped"] = parser.stats.bytes_dropped

    def ingest(self, frame: Frame) -> None:
        """Fold one frame into the dashboard state."""
        result = decode(frame)
        with self._lock:
            self._frame_times.append(frame.timestamp)
            key = frame.key

            if isinstance(result, UnknownFrame):
                self._stats["unknown"] += 1
                name = "unknown"
                known = False
                decoded: dict[str, Any] = {}
            else:
                spec = MESSAGE_MAP[key]
                name = spec.name
                known = True
                decoded = {}
                for reading in result:
                    self._stats["readings"] += 1
                    decoded[reading.field_name] = reading.value
                    previous = self._fields.get(reading.field_name)
                    self._fields[reading.field_name] = FieldSnapshot(
                        value=reading.value,
                        unit=reading.unit,
                        timestamp=reading.timestamp,
                        count=(previous.count + 1) if previous else 1,
                    )
                    if reading.field_name in CHARTED_FIELDS and isinstance(
                        reading.value, (int, float)
                    ):
                        history = self._series.setdefault(
                            reading.field_name, deque(maxlen=HISTORY_POINTS)
                        )
                        history.append((reading.timestamp, float(reading.value)))

                if key == (0x80, 0x05):
                    self._track.append(
                        (
                            frame.timestamp,
                            int(decoded.get("target_x_cm", 0)),
                            int(decoded.get("target_y_cm", 0)),
                            int(decoded.get("target_z_cm", 0)),
                        )
                    )

            snapshot = self._messages.get(key)
            self._messages[key] = MessageSnapshot(
                control=frame.control,
                command=frame.command,
                name=name,
                known=known,
                count=(snapshot.count + 1) if snapshot else 1,
                last_hex=frame.raw_hex,
                last_payload=frame.payload.hex(" "),
                last_seen=frame.timestamp,
            )

            self._log.append(
                {
                    "ts": frame.timestamp,
                    "control": frame.control,
                    "command": frame.command,
                    "name": name,
                    "known": known,
                    "valid": frame.valid_checksum,
                    "hex": frame.raw_hex,
                    "fields": {k: v for k, v in decoded.items()},
                }
            )

    # -- reader side -------------------------------------------------------

    def _frame_rate(self) -> float:
        if len(self._frame_times) < 2:
            return 0.0
        span = self._frame_times[-1] - self._frame_times[0]
        if span <= 0:
            return 0.0
        return (len(self._frame_times) - 1) / span

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "now": time.time(),
                "mode": self.mode,
                "port": self.port,
                "baud": self.baud,
                "connected": self.connected,
                "status_detail": self.status_detail,
                "reconnects": self.reconnects,
                "elapsed": time.time() - self.started,
                "frame_rate": self._frame_rate(),
                "stats": dict(self._stats),
                "fields": {
                    name: {
                        "value": fs.value,
                        "unit": fs.unit,
                        "ts": fs.timestamp,
                        "count": fs.count,
                        "label": ENUM_LABELS.get(name, {}).get(fs.value)
                        if isinstance(fs.value, int)
                        else None,
                    }
                    for name, fs in self._fields.items()
                },
                "series": {
                    name: [[round(ts, 3), value] for ts, value in points]
                    for name, points in self._series.items()
                },
                "track": [
                    [round(ts, 3), x, y, z] for ts, x, y, z in self._track
                ],
                "messages": [
                    {
                        "control": m.control,
                        "command": m.command,
                        "name": m.name,
                        "known": m.known,
                        "count": m.count,
                        "last_hex": m.last_hex,
                        "last_payload": m.last_payload,
                        "last_seen": m.last_seen,
                    }
                    for m in sorted(
                        self._messages.values(), key=lambda m: (m.control, m.command)
                    )
                ],
                "log": list(self._log),
                "known_pairs": [
                    {
                        "control": control,
                        "command": command,
                        "name": spec.name,
                        "unit": spec.unit,
                        "verified": spec.verified,
                    }
                    for (control, command), spec in sorted(MESSAGE_MAP.items())
                ],
            }


class RadarSource(threading.Thread):
    """Reads the serial port in the background and feeds a VizState."""

    def __init__(
        self,
        state: VizState,
        port: str,
        baud: int,
        sink: CsvSink | None = None,
    ) -> None:
        super().__init__(name="radar-source", daemon=True)
        self.state = state
        self.port = port
        self.baud = baud
        self.sink = sink
        self.parser = FrameParser()
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def run(self) -> None:
        attempt = 0
        while not self._stop.is_set():
            try:
                with serial.Serial(self.port, self.baud, timeout=0.2) as ser:
                    if attempt:
                        self.state.note_reconnect()
                    attempt = 0
                    self.parser.reset()
                    ser.reset_input_buffer()
                    self.state.set_connection(True, f"{self.port} @ {self.baud}")
                    self._pump(ser)
            except (serial.SerialException, OSError) as exc:
                delay = _RECONNECT_BACKOFF[min(attempt, len(_RECONNECT_BACKOFF) - 1)]
                attempt += 1
                self.state.set_connection(
                    False, f"{type(exc).__name__}: {exc} -- retry in {delay:.0f}s"
                )
                self._stop.wait(delay)

    def _pump(self, ser: serial.Serial) -> None:
        while not self._stop.is_set():
            chunk = ser.read(1024)
            if chunk:
                for frame in self.parser.feed(chunk):
                    self.state.ingest(frame)
                    if self.sink is not None and frame.valid_checksum:
                        result = decode(frame)
                        if not isinstance(result, UnknownFrame):
                            for reading in result:
                                self.sink.write(reading)
                self.state.sync_parser_stats(self.parser)


class ReplaySource(threading.Thread):
    """Replays a recorded CSV so the dashboard can be driven without hardware."""

    def __init__(self, state: VizState, csv_path: Path, speed: float = 1.0, loop: bool = True) -> None:
        super().__init__(name="radar-replay", daemon=True)
        self.state = state
        self.csv_path = csv_path
        self.speed = max(speed, 0.01)
        self.loop = loop
        self._stop = threading.Event()

    def stop(self) -> None:
        self._stop.set()

    def _frames(self) -> Iterator[tuple[float, bytes]]:
        """Yield (timestamp, raw_frame) once per distinct frame in the CSV.

        A frame that decoded to several fields occupies several rows; they all
        carry the same raw_hex, so collapse consecutive duplicates.
        """
        with self.csv_path.open(newline="", encoding="utf-8") as handle:
            previous: tuple[str, str] | None = None
            for row in csv.DictReader(handle):
                raw_hex = row.get("raw_hex", "")
                unix_ts = row.get("unix_ts", "")
                if not raw_hex:
                    continue
                marker = (unix_ts, raw_hex)
                if marker == previous:
                    continue
                previous = marker
                try:
                    yield float(unix_ts), bytes.fromhex(raw_hex.replace(" ", ""))
                except ValueError:
                    continue

    def run(self) -> None:
        # One parser for the life of the thread: looping the capture must not
        # reset the frame counters the dashboard header reads.
        parser = FrameParser()
        while not self._stop.is_set():
            self.state.set_connection(True, f"replaying {self.csv_path.name}")
            previous_ts: float | None = None
            emitted = 0

            for source_ts, raw in self._frames():
                if self._stop.is_set():
                    return
                if previous_ts is not None:
                    gap = (source_ts - previous_ts) / self.speed
                    if 0 < gap < 30:
                        self._stop.wait(gap)
                previous_ts = source_ts
                for frame in parser.feed(raw, timestamp=time.time()):
                    self.state.ingest(frame)
                    emitted += 1
                self.state.sync_parser_stats(parser)

            if emitted == 0:
                self.state.set_connection(False, f"no frames in {self.csv_path.name}")
                return
            if not self.loop:
                self.state.set_connection(False, "replay finished")
                return
            self._stop.wait(1.0)


def _make_handler(state: VizState, html: bytes) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        server_version = "r60a-visualize"

        def log_message(self, format: str, *args: Any) -> None:
            return  # keep the console clean for the summary line

        def _send(self, body: bytes, content_type: str) -> None:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                self._send(html, "text/html; charset=utf-8")
            elif path == "/api/state":
                payload = json.dumps(state.snapshot()).encode()
                self._send(payload, "application/json")
            elif path == "/events":
                self._stream()
            else:
                self.send_error(404, "not found")

        def _stream(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            try:
                while True:
                    payload = json.dumps(state.snapshot())
                    self.wfile.write(f"data: {payload}\n\n".encode())
                    self.wfile.flush()
                    time.sleep(0.2)
            except (BrokenPipeError, ConnectionResetError, OSError):
                return  # browser navigated away

    return Handler


def serve(state: VizState, host: str, http_port: int) -> ThreadingHTTPServer:
    html = DASHBOARD.read_bytes()
    server = ThreadingHTTPServer((host, http_port), _make_handler(state, html))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, name="http", daemon=True)
    thread.start()
    return server


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="r60a-visualize", description=__doc__)
    parser.add_argument("--port", default="auto", help="serial device, or 'auto'")
    parser.add_argument("--baud", default="auto", help="baud rate, or 'auto'")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind address")
    parser.add_argument("--http-port", type=int, default=8420, help="HTTP port")
    parser.add_argument(
        "--replay", help="replay a recorded CSV instead of reading the radar"
    )
    parser.add_argument(
        "--speed", type=float, default=1.0, help="replay speed multiplier"
    )
    parser.add_argument("--out", help="also append decoded readings to this CSV")
    parser.add_argument(
        "--probe-seconds", type=float, default=3.0, help="sample length per baud"
    )
    parser.add_argument(
        "--no-browser", action="store_true", help="do not open a browser window"
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)

    if not DASHBOARD.exists():
        print(f"error: dashboard asset missing at {DASHBOARD}", file=sys.stderr)
        return 2

    state = VizState()
    source: RadarSource | ReplaySource

    if args.replay:
        csv_path = Path(args.replay)
        if not csv_path.exists():
            print(f"error: no such capture: {csv_path}", file=sys.stderr)
            return 2
        state.mode = "replay"
        state.port = str(csv_path)
        source = ReplaySource(state, csv_path, args.speed)
    else:
        try:
            port = args.port if args.port != "auto" else autodetect_port()
            if args.baud == "auto":

                def report(probe: BaudProbe) -> None:
                    detail = probe.error or (
                        f"{probe.bytes_read} bytes, {probe.frames_ok} frames"
                    )
                    print(f"[discovery] {probe.baud:>7}: {detail}")

                baud = autodetect_baud(port, CANDIDATE_BAUDS, args.probe_seconds, report)
            else:
                baud = int(args.baud)
        except DiscoveryError as exc:
            print(f"error: {exc}", file=sys.stderr)
            print("hint: use --replay data/session.csv to view a recorded session.")
            return 2
        state.mode = "live"
        state.port = port
        state.baud = baud
        sink = CsvSink(Path(args.out)) if args.out else None
        source = RadarSource(state, port, baud, sink)

    try:
        server = serve(state, args.host, args.http_port)
    except OSError as exc:
        print(f"error: cannot bind {args.host}:{args.http_port}: {exc}", file=sys.stderr)
        return 2

    source.start()
    url = f"http://{args.host}:{args.http_port}/"
    print(f"[visualize] dashboard: {url}")
    print("[visualize] Ctrl-C to stop")
    if not args.no_browser:
        webbrowser.open(url)

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[visualize] stopping")
    finally:
        source.stop()
        server.shutdown()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
