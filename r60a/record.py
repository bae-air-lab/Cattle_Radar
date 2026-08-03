"""Capture tool for the MicRadar R60A.

    python -m r60a.record --port auto --baud auto --out data/session.csv --duration 60

Streams frames off the serial port, writes one CSV row per decoded field, shows
a live terminal view, reconnects with backoff if the port drops, and prints a
session summary on Ctrl-C.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence, TextIO

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
    DecodeStats,
    Frame,
    FrameParser,
    Reading,
    UnknownFrame,
    decode,
)

__all__ = ["main", "Recorder", "SessionState"]

CSV_COLUMNS: tuple[str, ...] = (
    "unix_ts",
    "iso_time",
    "control",
    "command",
    "field_name",
    "value",
    "raw_hex",
)

_RECONNECT_BACKOFF: tuple[float, ...] = (0.5, 1.0, 2.0, 4.0, 8.0, 10.0)


def _iso(ts: float) -> str:
    return datetime.fromtimestamp(ts, timezone.utc).astimezone().isoformat(
        timespec="milliseconds"
    )


@dataclass
class FieldState:
    """Latest value of one decoded field, for the live display."""

    value: object
    unit: str
    timestamp: float
    count: int = 0

    def render(self, name: str) -> str:
        labels = ENUM_LABELS.get(name)
        if labels is not None and isinstance(self.value, int):
            return f"{self.value} ({labels.get(self.value, '?')})"
        if isinstance(self.value, float):
            return f"{self.value:g}"
        return str(self.value)


@dataclass
class SessionState:
    """Everything the live display and the final summary need."""

    started: float = field(default_factory=time.time)
    rows_written: int = 0
    reconnects: int = 0
    port: str = ""
    baud: int = 0
    fields: dict[str, FieldState] = field(default_factory=dict)
    decode_stats: DecodeStats = field(default_factory=DecodeStats)
    _frame_times: deque[float] = field(default_factory=lambda: deque(maxlen=128))

    def note_frame(self, ts: float) -> None:
        self._frame_times.append(ts)

    def note_reading(self, reading: Reading) -> None:
        state = self.fields.get(reading.field_name)
        count = state.count + 1 if state is not None else 1
        self.fields[reading.field_name] = FieldState(
            value=reading.value,
            unit=reading.unit,
            timestamp=reading.timestamp,
            count=count,
        )

    @property
    def elapsed(self) -> float:
        return time.time() - self.started

    @property
    def frames_per_sec(self) -> float:
        """Rate over the recent window, so it reflects now rather than the average."""
        if len(self._frame_times) < 2:
            return 0.0
        span = self._frame_times[-1] - self._frame_times[0]
        if span <= 0:
            return 0.0
        return (len(self._frame_times) - 1) / span


class LiveDisplay:
    """Plain-ANSI live view.  Redraws in place; degrades to nothing when the
    output is not a TTY (piping to a file stays readable)."""

    def __init__(self, stream: TextIO = sys.stdout, enabled: bool = True) -> None:
        self._stream = stream
        self._enabled = enabled and stream.isatty()
        self._lines_drawn = 0
        self._last_draw = 0.0

    @property
    def enabled(self) -> bool:
        return self._enabled

    def draw(self, state: SessionState, parser: FrameParser, force: bool = False) -> None:
        if not self._enabled:
            return
        now = time.monotonic()
        if not force and now - self._last_draw < 0.2:
            return
        self._last_draw = now

        lines = self._compose(state, parser)
        out = self._stream
        if self._lines_drawn:
            out.write(f"\033[{self._lines_drawn}A")
        for line in lines:
            out.write("\033[2K" + line + "\n")
        # Clear any leftovers from a previously taller frame.
        for _ in range(max(0, self._lines_drawn - len(lines))):
            out.write("\033[2K\n")
        self._lines_drawn = max(len(lines), self._lines_drawn)
        out.flush()

    def _compose(self, state: SessionState, parser: FrameParser) -> list[str]:
        stats = parser.stats
        lines = [
            f"R60A  {state.port} @ {state.baud}  "
            f"elapsed {state.elapsed:6.1f}s  {state.frames_per_sec:5.2f} frames/s",
            f"frames {stats.frames_ok:<6d} bad-checksum {stats.frames_bad_checksum:<5d} "
            f"unknown {state.decode_stats.unknown_frames:<5d} "
            f"dropped {stats.bytes_dropped:<6d} rows {state.rows_written:<6d} "
            f"reconnects {state.reconnects}",
            "-" * 72,
        ]
        if not state.fields:
            lines.append("  waiting for data ...")
        else:
            width = max(len(name) for name in state.fields)
            for name in sorted(state.fields):
                fs = state.fields[name]
                age = time.time() - fs.timestamp
                lines.append(
                    f"  {name:<{width}}  {fs.render(name):>18}  {fs.unit:<20} "
                    f"n={fs.count:<6d} {age:4.1f}s ago"
                )
        return lines

    def finish(self) -> None:
        if self._enabled and self._lines_drawn:
            self._stream.write("\n")
            self._stream.flush()
            self._lines_drawn = 0


class CsvSink:
    """Buffered CSV writer that is safe to close twice."""

    def __init__(self, path: Path, flush_every: float = 2.0) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self._path = path
        self._handle = path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._handle)
        self._writer.writerow(CSV_COLUMNS)
        self._flush_every = flush_every
        self._last_flush = time.monotonic()
        self._closed = False

    @property
    def path(self) -> Path:
        return self._path

    def write(self, reading: Reading) -> None:
        self._writer.writerow(
            (
                f"{reading.timestamp:.6f}",
                _iso(reading.timestamp),
                f"0x{reading.control:02x}",
                f"0x{reading.command:02x}",
                reading.field_name,
                reading.value,
                reading.raw_hex,
            )
        )
        now = time.monotonic()
        if now - self._last_flush >= self._flush_every:
            self._handle.flush()
            self._last_flush = now

    def close(self) -> None:
        if not self._closed:
            self._handle.flush()
            self._handle.close()
            self._closed = True


class Recorder:
    """Owns the serial connection, the parser and the sinks."""

    def __init__(
        self,
        port: str,
        baud: int,
        sink: CsvSink | None,
        display: LiveDisplay,
        keep_bad_checksum: bool = False,
        verbose_unknown: bool = False,
    ) -> None:
        self.port = port
        self.baud = baud
        self.sink = sink
        self.display = display
        self.keep_bad_checksum = keep_bad_checksum
        self.verbose_unknown = verbose_unknown
        self.parser = FrameParser()
        self.state = SessionState(port=port, baud=baud)

    def run(self, duration: float | None) -> None:
        """Stream until ``duration`` seconds elapse, or forever if None.

        Reconnects with backoff on a dropped port; KeyboardInterrupt is left
        to the caller so it can print the summary.
        """
        deadline = None if duration is None else time.monotonic() + duration
        attempt = 0

        while deadline is None or time.monotonic() < deadline:
            try:
                with serial.Serial(self.port, self.baud, timeout=0.2) as ser:
                    if attempt:
                        self.state.reconnects += 1
                    attempt = 0
                    self.parser.reset()
                    ser.reset_input_buffer()
                    self._pump(ser, deadline)
                    return
            except (serial.SerialException, OSError) as exc:
                self.display.finish()
                delay = _RECONNECT_BACKOFF[min(attempt, len(_RECONNECT_BACKOFF) - 1)]
                attempt += 1
                remaining = None if deadline is None else deadline - time.monotonic()
                if remaining is not None and remaining <= 0:
                    return
                print(
                    f"[port] {type(exc).__name__}: {exc} -- retrying in {delay:.1f}s",
                    file=sys.stderr,
                )
                sleep_for = delay if remaining is None else min(delay, remaining)
                time.sleep(max(0.0, sleep_for))

    def _pump(self, ser: serial.Serial, deadline: float | None) -> None:
        while deadline is None or time.monotonic() < deadline:
            chunk = ser.read(1024)
            if chunk:
                for frame in self.parser.feed(chunk):
                    self._handle(frame)
            self.display.draw(self.state, self.parser)

    def _handle(self, frame: Frame) -> None:
        self.state.note_frame(frame.timestamp)

        if not frame.valid_checksum and not self.keep_bad_checksum:
            return  # already counted by the parser

        result = decode(frame)
        if isinstance(result, UnknownFrame):
            self.state.decode_stats.note_unknown(result)
            if self.verbose_unknown:
                self.display.finish()
                print(
                    f"[unknown] 0x{result.control:02x}/0x{result.command:02x}  "
                    f"{result.raw_hex}",
                    file=sys.stderr,
                )
            return

        for reading in result:
            self.state.decode_stats.readings += 1
            self.state.note_reading(reading)
            if self.sink is not None:
                self.sink.write(reading)
                self.state.rows_written += 1

    def summary(self) -> str:
        stats = self.parser.stats
        state = self.state
        lines = [
            "",
            "=== session summary ===",
            f"  port / baud     : {state.port} @ {state.baud}",
            f"  duration        : {state.elapsed:.1f} s",
            f"  frames ok       : {stats.frames_ok}",
            f"  bad checksum    : {stats.frames_bad_checksum}",
            f"  unknown frames  : {state.decode_stats.unknown_frames}",
            f"  readings        : {state.decode_stats.readings}",
            f"  bytes dropped   : {stats.bytes_dropped} (resyncs: {stats.resyncs})",
            f"  reconnects      : {state.reconnects}",
        ]
        if state.elapsed > 0:
            lines.append(f"  average rate    : {stats.frames_ok / state.elapsed:.2f} frames/s")
        if self.sink is not None:
            lines.append(f"  csv rows        : {state.rows_written} -> {self.sink.path}")

        if state.decode_stats.unknown_pairs:
            lines.append("  unknown (control, command) pairs:")
            for (control, command), count in sorted(
                state.decode_stats.unknown_pairs.items()
            ):
                lines.append(f"    0x{control:02x} 0x{command:02x}  x{count}")

        if state.fields:
            lines.append("  last values:")
            width = max(len(name) for name in state.fields)
            for name in sorted(state.fields):
                fs = state.fields[name]
                lines.append(
                    f"    {name:<{width}}  {fs.render(name):>18}  {fs.unit:<20} n={fs.count}"
                )
        return "\n".join(lines)


def _resolve_port(requested: str) -> str:
    if requested != "auto":
        return requested
    port = autodetect_port()
    print(f"[discovery] port: {port}")
    return port


def _resolve_baud(requested: str, port: str, seconds: float) -> int:
    if requested != "auto":
        return int(requested)

    def report(probe: BaudProbe) -> None:
        detail = probe.error or (
            f"{probe.bytes_read} bytes, {probe.frames_ok} frames, "
            f"{probe.frames_bad} bad"
        )
        print(f"[discovery] {probe.baud:>7}: {detail}")

    baud = autodetect_baud(port, CANDIDATE_BAUDS, seconds, on_probe=report)
    print(f"[discovery] baud: {baud}")
    return baud


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="r60a-record", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", default="auto", help="serial device, or 'auto'")
    parser.add_argument("--baud", default="auto", help="baud rate, or 'auto'")
    parser.add_argument(
        "--out", default="data/session.csv", help="CSV output path ('none' to skip)"
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=60.0,
        help="seconds to record; 0 or negative means run until Ctrl-C",
    )
    parser.add_argument(
        "--probe-seconds",
        type=float,
        default=3.0,
        help="sample length per baud during --baud auto",
    )
    parser.add_argument("--no-live", action="store_true", help="disable the live display")
    parser.add_argument(
        "--keep-bad",
        action="store_true",
        help="decode and log frames whose checksum failed",
    )
    parser.add_argument(
        "--show-unknown",
        action="store_true",
        help="print unrecognised (control, command) frames as they arrive",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)

    try:
        port = _resolve_port(args.port)
        baud = _resolve_baud(args.baud, port, args.probe_seconds)
    except DiscoveryError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    sink = None if args.out.lower() == "none" else CsvSink(Path(args.out))
    display = LiveDisplay(enabled=not args.no_live)
    recorder = Recorder(
        port=port,
        baud=baud,
        sink=sink,
        display=display,
        keep_bad_checksum=args.keep_bad,
        verbose_unknown=args.show_unknown,
    )

    duration = args.duration if args.duration and args.duration > 0 else None
    if duration is None:
        print("[record] recording until Ctrl-C")

    interrupted = False
    try:
        recorder.run(duration)
    except KeyboardInterrupt:
        interrupted = True
    finally:
        display.finish()
        if sink is not None:
            sink.close()

    if interrupted:
        print("[record] interrupted")
    print(recorder.summary())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
