"""Session recorder for the MicRadar R60A, with a small desktop UI.

    python -m r60a.recorder
    python -m r60a.recorder --port /dev/ttyUSB0 --baud 115200 --outdir data

Type a subject ID, press Start.  The recorder reads the radar for a warm-up
period (15 s by default) *without* keeping anything -- the module needs that
long to lock on and converge -- then records one sample per second until Stop,
and writes ``<outdir>/<id>.json``.

Every sample carries a ``valid`` flag from :class:`VitalsGate`.  The R60A
free-runs: its vital-signs engine keeps publishing heart-rate and breathing
frames when nobody is in front of the antenna, holding or drifting around the
last value it locked onto.  Those numbers are not measurements, and nothing in
the frame itself says so, so the gate reconstructs the missing context --
occupancy, range, plausibility, and whether the value is actually still moving
-- and marks samples that fail.  See :class:`GateConfig` for the thresholds.
"""

from __future__ import annotations

import argparse
import json
import re
import statistics
import sys
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence

import serial

from .discovery import (
    CANDIDATE_BAUDS,
    BaudProbe,
    DiscoveryError,
    autodetect_baud,
    autodetect_port,
)
from .protocol import ENUM_LABELS, Frame, FrameParser, UnknownFrame, decode

try:  # pragma: no cover - exercised only on a headless install
    import tkinter as tk
    from tkinter import font as tkfont
    from tkinter import messagebox, ttk
except ImportError:  # tkinter is optional at import time so the gate stays testable
    tk = None  # type: ignore[assignment]

__all__ = [
    "main",
    "GateConfig",
    "Verdict",
    "VitalsGate",
    "Sample",
    "RecorderCore",
    "RadarReader",
]

SCHEMA = "r60a.recorder/1"

DEFAULT_WARMUP_S = 15.0
DEFAULT_SAMPLE_HZ = 1.0

#: Fields carried on every sample, in output order.
SAMPLE_FIELDS: tuple[str, ...] = (
    "heart_rate_bpm",
    "breathing_rate_bpm",
    "target_distance_cm",
    "target_x_cm",
    "target_y_cm",
    "target_z_cm",
    "motion_state",
    "body_movement",
    "presence",
)

_RECONNECT_BACKOFF: tuple[float, ...] = (0.5, 1.0, 2.0, 4.0, 8.0, 10.0)

_ID_SAFE = re.compile(r"[^A-Za-z0-9._-]+")

#: Human-readable explanations, shown in the UI and stored in the summary.
REJECT_REASONS: dict[str, str] = {
    "disconnected": "serial link down",
    "no_target": "no occupancy evidence (motion/distance)",
    "no_distance": "no recent target-distance frame",
    "target_too_far": "target beyond the configured range",
    "heart_rate_stale": "no recent heart-rate frame",
    "heart_rate_zero": "heart rate reported as 0",
    "heart_rate_out_of_range": "heart rate outside the plausible band",
    "heart_rate_frozen": "heart rate held constant -- latched, not measured",
    "breathing_rate_stale": "no recent breathing-rate frame",
    "breathing_rate_zero": "breathing rate reported as 0",
    "breathing_rate_out_of_range": "breathing rate outside the plausible band",
    "breathing_rate_frozen": "breathing rate held constant -- latched, not measured",
}


def _iso(ts: float) -> str:
    return datetime.fromtimestamp(ts, timezone.utc).astimezone().isoformat(
        timespec="milliseconds"
    )


def sanitise_id(text: str) -> str:
    """Turn free text into something safe to use as a filename stem."""
    cleaned = _ID_SAFE.sub("_", text.strip()).strip("._-")
    return cleaned


# ---------------------------------------------------------------------------
# Validity gating
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class GateConfig:
    """Thresholds that decide whether a vitals sample is believable.

    Defaults are deliberately wide enough to cover a resting adult (HR 55-90,
    breathing 12-25) and cattle (HR 48-84, breathing 26-50); narrow them for a
    single species when you want a stricter filter.
    """

    enabled: bool = True
    max_distance_cm: float = 250.0
    heart_min: int = 35
    heart_max: int = 150
    breath_min: int = 5
    breath_max: int = 60
    #: A vitals frame older than this is not a live reading any more.
    stale_after_s: float = 6.0
    #: How long motion/distance/presence evidence stays good for.
    occupancy_timeout_s: float = 6.0
    #: An unchanging value over this long is the module holding its last lock.
    freeze_window_s: float = 20.0
    freeze_min_updates: int = 6

    def to_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "max_distance_cm": self.max_distance_cm,
            "heart_rate_bpm_range": [self.heart_min, self.heart_max],
            "breathing_rate_bpm_range": [self.breath_min, self.breath_max],
            "stale_after_s": self.stale_after_s,
            "occupancy_timeout_s": self.occupancy_timeout_s,
            "freeze_window_s": self.freeze_window_s,
            "freeze_min_updates": self.freeze_min_updates,
        }


@dataclass(frozen=True)
class Verdict:
    """Outcome of gating one instant of radar state."""

    valid: bool
    occupied: bool
    reasons: tuple[str, ...] = ()

    @property
    def summary(self) -> str:
        if self.valid:
            return "valid"
        return ", ".join(REJECT_REASONS.get(r, r) for r in self.reasons)


class VitalsGate:
    """Decides whether the current heart/breathing numbers mean anything.

    Four independent checks, because no single one catches every failure:

    1. **Occupancy.**  Vitals without a target are noise.  ``0x80 0x01``
       (presence) has never been seen on this hardware, so occupancy falls back
       to a fresh non-zero motion state or a fresh in-range distance -- and
       upgrades to the reported presence bit automatically if the module ever
       does emit one.
    2. **Range.**  A target further out than ``max_distance_cm`` is outside the
       zone the vitals engine can resolve.
    3. **Plausibility.**  Zero, and anything outside the configured band.
    4. **Freeze.**  A real cardiac or respiratory estimate jitters by a bpm or
       two from frame to frame.  A value that is bit-identical across
       ``freeze_window_s`` is the module replaying its last lock, which is
       exactly what it does when the subject walks away or is occluded.
    """

    _TRACKED: tuple[str, ...] = ("heart_rate_bpm", "breathing_rate_bpm")

    def __init__(self, config: GateConfig | None = None) -> None:
        self.config = config or GateConfig()
        self._latest: dict[str, tuple[float, Any]] = {}
        self._history: dict[str, deque[tuple[float, float]]] = {
            name: deque(maxlen=512) for name in self._TRACKED
        }
        self.presence_reported = False

    # -- ingest ------------------------------------------------------------

    def note(self, field_name: str, value: Any, timestamp: float) -> None:
        self._latest[field_name] = (timestamp, value)
        if field_name == "presence":
            self.presence_reported = True
        history = self._history.get(field_name)
        if history is not None and isinstance(value, (int, float)):
            history.append((timestamp, float(value)))

    def reset(self) -> None:
        """Forget history.  Called on reconnect: values either side of a drop
        are not a continuous series, and comparing them would fake a freeze."""
        self._latest.clear()
        for history in self._history.values():
            history.clear()

    # -- queries -----------------------------------------------------------

    def latest(self, field_name: str) -> tuple[float, Any] | None:
        return self._latest.get(field_name)

    def age(self, field_name: str, now: float) -> float | None:
        entry = self._latest.get(field_name)
        return None if entry is None else now - entry[0]

    def fresh(self, field_name: str, now: float, within: float) -> Any | None:
        """The value if it arrived recently enough, else None."""
        entry = self._latest.get(field_name)
        if entry is None or now - entry[0] > within:
            return None
        return entry[1]

    def frozen(self, field_name: str, now: float) -> bool:
        cfg = self.config
        history = self._history.get(field_name)
        if not history:
            return False
        window: list[tuple[float, float]] = []
        for ts, value in reversed(history):
            if now - ts > cfg.freeze_window_s:
                break
            window.append((ts, value))
        if len(window) < cfg.freeze_min_updates:
            return False
        span = window[0][0] - window[-1][0]
        if span < cfg.freeze_window_s * 0.75:
            return False  # not enough elapsed time to call it held
        return len({value for _, value in window}) == 1

    def occupied(self, now: float) -> bool:
        cfg = self.config
        presence = self.fresh("presence", now, cfg.occupancy_timeout_s)
        if presence is not None:
            return bool(presence)  # reported beats inferred
        motion = self.fresh("motion_state", now, cfg.occupancy_timeout_s)
        if motion is not None and motion > 0:
            return True
        distance = self.fresh("target_distance_cm", now, cfg.occupancy_timeout_s)
        return distance is not None and 0 < distance <= cfg.max_distance_cm

    def evaluate(self, now: float, connected: bool = True) -> Verdict:
        cfg = self.config
        reasons: list[str] = []

        if not connected:
            reasons.append("disconnected")

        occupied = self.occupied(now)
        if not occupied:
            reasons.append("no_target")

        distance = self.fresh("target_distance_cm", now, cfg.occupancy_timeout_s)
        if distance is None:
            reasons.append("no_distance")
        elif distance > cfg.max_distance_cm:
            reasons.append("target_too_far")

        for name, low, high in (
            ("heart_rate_bpm", cfg.heart_min, cfg.heart_max),
            ("breathing_rate_bpm", cfg.breath_min, cfg.breath_max),
        ):
            prefix = "heart_rate" if name.startswith("heart") else "breathing_rate"
            value = self.fresh(name, now, cfg.stale_after_s)
            if value is None:
                reasons.append(f"{prefix}_stale")
                continue
            if value == 0:
                reasons.append(f"{prefix}_zero")
            elif not (low <= value <= high):
                reasons.append(f"{prefix}_out_of_range")
            if self.frozen(name, now):
                reasons.append(f"{prefix}_frozen")

        if not cfg.enabled:
            return Verdict(valid=True, occupied=occupied, reasons=tuple(reasons))
        return Verdict(valid=not reasons, occupied=occupied, reasons=tuple(reasons))


# ---------------------------------------------------------------------------
# Session state
# ---------------------------------------------------------------------------


@dataclass
class Sample:
    """One time-sliced snapshot of the radar state."""

    unix_ts: float
    elapsed: float
    values: dict[str, Any]
    occupied: bool
    valid: bool
    reject: tuple[str, ...]

    def to_dict(self) -> dict[str, Any]:
        row: dict[str, Any] = {
            "t": round(self.elapsed, 3),
            "unix_ts": round(self.unix_ts, 3),
            "iso_time": _iso(self.unix_ts),
        }
        row.update({name: self.values.get(name) for name in SAMPLE_FIELDS})
        row["occupied"] = self.occupied
        row["valid"] = self.valid
        row["reject"] = list(self.reject)
        return row


class RecorderCore:
    """Parser, gate and session state, shared between the reader thread and the UI.

    Every public method takes the lock, so the UI can poll :meth:`snapshot`
    while the reader thread is feeding frames in.
    """

    def __init__(
        self,
        gate_config: GateConfig | None = None,
        warmup_s: float = DEFAULT_WARMUP_S,
        sample_hz: float = DEFAULT_SAMPLE_HZ,
        keep_invalid: bool = True,
    ) -> None:
        self.gate = VitalsGate(gate_config)
        self.warmup_s = warmup_s
        self.sample_interval = 1.0 / max(sample_hz, 0.01)
        self.keep_invalid = keep_invalid

        self.parser = FrameParser()
        self._lock = threading.RLock()

        self.port = ""
        self.baud = 0
        self.connected = False
        self.status_detail = "starting"

        self.phase = "idle"  # idle | warmup | recording
        self.subject_id = ""
        self.phase_started = 0.0
        self.record_started = 0.0
        self.warmup_ends = 0.0
        self._next_sample = 0.0

        self.samples: list[Sample] = []
        self.rejected = 0
        self.last_verdict = Verdict(valid=False, occupied=False, reasons=("no_target",))
        self.last_valid_ts: float | None = None
        self.unknown_frames = 0
        self.reconnects = 0
        self._warmup_frames = 0

    # -- reader-thread side ------------------------------------------------

    def set_connection(self, connected: bool, detail: str) -> None:
        with self._lock:
            if self.connected and not connected:
                self.gate.reset()  # do not compare across a gap
            self.connected = connected
            self.status_detail = detail

    def note_reconnect(self) -> None:
        with self._lock:
            self.reconnects += 1

    def ingest(self, frame: Frame) -> None:
        """Fold one frame into the gate.  Bad-checksum frames are dropped:
        a corrupted byte would otherwise register as a plausible vital."""
        if not frame.valid_checksum:
            return
        result = decode(frame)
        with self._lock:
            if isinstance(result, UnknownFrame):
                self.unknown_frames += 1
                return
            for reading in result:
                self.gate.note(reading.field_name, reading.value, reading.timestamp)
            if self.phase == "warmup":
                self._warmup_frames += 1

    def tick(self, now: float | None = None) -> None:
        """Advance the session clock: warm-up expiry and sample emission.

        Called from the reader loop several times a second, with or without
        new data -- a sample must still be emitted (and marked invalid) when
        the radar goes quiet.
        """
        now = time.time() if now is None else now
        with self._lock:
            verdict = self.gate.evaluate(now, connected=self.connected)
            self.last_verdict = verdict
            if verdict.valid:
                self.last_valid_ts = now

            if self.phase == "warmup" and now >= self.warmup_ends:
                self.phase = "recording"
                self.record_started = now
                self._next_sample = now

            if self.phase != "recording" or now < self._next_sample:
                return

            self._emit(now, verdict)
            self._next_sample += self.sample_interval
            if self._next_sample < now:  # fell behind; resynchronise
                self._next_sample = now + self.sample_interval

    def _emit(self, now: float, verdict: Verdict) -> None:
        values = {
            name: (entry[1] if (entry := self.gate.latest(name)) else None)
            for name in SAMPLE_FIELDS
        }
        sample = Sample(
            unix_ts=now,
            elapsed=now - self.record_started,
            values=values,
            occupied=verdict.occupied,
            valid=verdict.valid,
            reject=verdict.reasons,
        )
        if not sample.valid:
            self.rejected += 1
        if sample.valid or self.keep_invalid:
            self.samples.append(sample)

    # -- UI side -----------------------------------------------------------

    def start(self, subject_id: str) -> None:
        with self._lock:
            now = time.time()
            self.subject_id = subject_id
            self.phase = "warmup"
            self.phase_started = now
            self.warmup_ends = now + self.warmup_s
            self.record_started = 0.0
            self.samples = []
            self.rejected = 0
            self.last_valid_ts = None
            self._warmup_frames = 0

    def cancel(self) -> None:
        with self._lock:
            self.phase = "idle"

    def stop(self) -> dict[str, Any]:
        """End the session and return the document to write."""
        with self._lock:
            now = time.time()
            document = self._document(now)
            self.phase = "idle"
            return document

    def _document(self, now: float) -> dict[str, Any]:
        started = self.record_started or now
        rows = [s.to_dict() for s in self.samples]
        valid = [s for s in self.samples if s.valid]
        return {
            "schema": SCHEMA,
            "id": self.subject_id,
            "device": {
                "port": self.port,
                "baud": self.baud,
                "connected_at_stop": self.connected,
            },
            "session": {
                "warmup_s": self.warmup_s,
                "warmup_started_iso": _iso(self.phase_started),
                "recording_started_iso": _iso(started),
                "recording_stopped_iso": _iso(now),
                "recording_started_unix": round(started, 3),
                "recording_stopped_unix": round(now, 3),
                "duration_s": round(max(0.0, now - started), 3),
                "sample_interval_s": self.sample_interval,
                "keep_invalid_samples": self.keep_invalid,
            },
            "gate": self.gate.config.to_dict(),
            "quality": self._quality(valid),
            "summary": self._stats(valid),
            "samples": rows,
        }

    def _quality(self, valid: Sequence[Sample]) -> dict[str, Any]:
        reasons: Counter[str] = Counter()
        for sample in self.samples:
            reasons.update(sample.reject)
        total = len(self.samples) + (0 if self.keep_invalid else self.rejected)
        stats = self.parser.stats
        return {
            "samples_total": total,
            "samples_valid": len(valid),
            "samples_rejected": self.rejected,
            "valid_fraction": round(len(valid) / total, 4) if total else 0.0,
            "presence_field_reported": self.gate.presence_reported,
            "reject_reasons": {
                reason: {"count": count, "meaning": REJECT_REASONS.get(reason, "")}
                for reason, count in reasons.most_common()
            },
            "frames_ok": stats.frames_ok,
            "frames_bad_checksum": stats.frames_bad_checksum,
            "unknown_frames": self.unknown_frames,
            "bytes_dropped": stats.bytes_dropped,
            "reconnects": self.reconnects,
        }

    @staticmethod
    def _stats(valid: Sequence[Sample]) -> dict[str, Any]:
        """Descriptive stats over the *valid* samples only -- averaging in the
        rejected ones is how a free-running radar ends up in a results table."""
        out: dict[str, Any] = {}
        for name in ("heart_rate_bpm", "breathing_rate_bpm", "target_distance_cm"):
            series = [
                float(s.values[name])
                for s in valid
                if isinstance(s.values.get(name), (int, float))
            ]
            if not series:
                out[name] = None
                continue
            out[name] = {
                "n": len(series),
                "mean": round(statistics.fmean(series), 2),
                "median": round(statistics.median(series), 2),
                "stdev": round(statistics.pstdev(series), 2) if len(series) > 1 else 0.0,
                "min": min(series),
                "max": max(series),
            }
        return out

    def snapshot(self) -> dict[str, Any]:
        """Everything the UI redraws from."""
        with self._lock:
            now = time.time()
            verdict = self.last_verdict
            fields: dict[str, Any] = {}
            for name in SAMPLE_FIELDS:
                entry = self.gate.latest(name)
                if entry is None:
                    fields[name] = None
                    continue
                ts, value = entry
                fields[name] = {
                    "value": value,
                    "age": now - ts,
                    "label": ENUM_LABELS.get(name, {}).get(value)
                    if isinstance(value, int)
                    else None,
                }
            return {
                "now": now,
                "phase": self.phase,
                "subject_id": self.subject_id,
                "connected": self.connected,
                "status_detail": self.status_detail,
                "port": self.port,
                "baud": self.baud,
                "warmup_remaining": max(0.0, self.warmup_ends - now)
                if self.phase == "warmup"
                else 0.0,
                "warmup_frames": self._warmup_frames,
                "elapsed": (now - self.record_started) if self.phase == "recording" else 0.0,
                "samples": len(self.samples),
                "valid": len(self.samples) - self.rejected if self.keep_invalid else len(self.samples),
                "rejected": self.rejected,
                "last_valid_age": (now - self.last_valid_ts)
                if self.last_valid_ts is not None
                else None,
                "verdict": {
                    "valid": verdict.valid,
                    "occupied": verdict.occupied,
                    "reasons": list(verdict.reasons),
                    "summary": verdict.summary,
                },
                "fields": fields,
                "frames_ok": self.parser.stats.frames_ok,
                "bad_checksum": self.parser.stats.frames_bad_checksum,
            }


# ---------------------------------------------------------------------------
# Serial reader
# ---------------------------------------------------------------------------


class RadarReader(threading.Thread):
    """Resolves the port, streams frames into a RecorderCore, reconnects."""

    def __init__(
        self,
        core: RecorderCore,
        port: str = "auto",
        baud: str = "auto",
        probe_seconds: float = 3.0,
    ) -> None:
        super().__init__(name="radar-reader", daemon=True)
        self.core = core
        self.requested_port = port
        self.requested_baud = baud
        self.probe_seconds = probe_seconds
        self._stop = threading.Event()
        self._baud: int | None = None

    def stop(self) -> None:
        self._stop.set()

    def _resolve(self) -> tuple[str, int]:
        port = self.requested_port
        if port == "auto":
            self.core.set_connection(False, "searching for a USB-serial adapter ...")
            port = autodetect_port()
        if self.requested_baud != "auto":
            return port, int(self.requested_baud)
        if self._baud is not None:
            return port, self._baud  # probed once; do not pay for it on every retry

        def report(probe: BaudProbe) -> None:
            self.core.set_connection(False, f"probing {probe.baud} baud ...")

        self.core.set_connection(False, f"probing baud rates on {port} ...")
        self._baud = autodetect_baud(port, CANDIDATE_BAUDS, self.probe_seconds, report)
        return port, self._baud

    def run(self) -> None:
        """Discover, stream, and keep retrying.

        Discovery failure is not fatal: the radar may simply not be plugged in
        yet, or not be transmitting.  Retrying means the window comes to life
        when it does, rather than needing a restart.
        """
        attempt = 0
        while not self._stop.is_set():
            try:
                port, baud = self._resolve()
            except DiscoveryError as exc:
                self._backoff(attempt, str(exc))
                attempt += 1
                continue

            self.core.port = port
            self.core.baud = baud
            try:
                with serial.Serial(port, baud, timeout=0.2) as ser:
                    if attempt:
                        self.core.note_reconnect()
                    attempt = 0
                    self.core.parser.reset()
                    ser.reset_input_buffer()
                    self.core.set_connection(True, f"{port} @ {baud}")
                    self._pump(ser)
            except (serial.SerialException, OSError) as exc:
                self._backoff(attempt, f"{type(exc).__name__}: {exc}")
                attempt += 1

    def _backoff(self, attempt: int, detail: str) -> None:
        """Wait out a failure while keeping the session clock running, so
        samples during an outage are recorded and flagged rather than missing."""
        delay = _RECONNECT_BACKOFF[min(attempt, len(_RECONNECT_BACKOFF) - 1)]
        self.core.set_connection(False, f"{detail} -- retry in {delay:.0f}s")
        deadline = time.monotonic() + delay
        while not self._stop.is_set() and time.monotonic() < deadline:
            self.core.tick()
            self._stop.wait(0.2)

    def _pump(self, ser: serial.Serial) -> None:
        while not self._stop.is_set():
            chunk = ser.read(1024)
            if chunk:
                for frame in self.core.parser.feed(chunk):
                    self.core.ingest(frame)
            self.core.tick()


# ---------------------------------------------------------------------------
# UI
# ---------------------------------------------------------------------------

_OK = "#1a7f37"
_WARN = "#b26a00"
_BAD = "#b3261e"
_MUTED = "#5f6368"

#: Added to every font in the window, named and explicit alike.
FONT_BUMP = 3


def enlarge_fonts(root: "tk.Misc", delta: int = FONT_BUMP) -> None:
    """Grow Tk's named fonts, which back every widget that does not ask for a
    font of its own -- buttons, the entry, frame titles.

    A negative size means pixels and a positive one means points, so the sign
    decides which way makes the text bigger.
    """
    for name in tkfont.names(root):
        font = tkfont.nametofont(name, root)
        size = font.cget("size")
        if not size:
            continue
        font.configure(size=size + delta if size > 0 else size - delta)


class RecorderApp:
    """Tk front end.  Owns no radar state -- it polls RecorderCore.snapshot()."""

    POLL_MS = 150

    def __init__(self, root: "tk.Tk", core: RecorderCore, outdir: Path) -> None:
        self.root = root
        self.core = core
        self.outdir = outdir

        root.title("R60A session recorder")
        root.minsize(620, 520)
        root.columnconfigure(0, weight=1)
        enlarge_fonts(root)

        self._build_header()
        self._build_status()
        self._build_readings()
        self._build_footer()

        root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._poll()

    # -- layout ------------------------------------------------------------

    def _build_header(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Session", padding=10)
        frame.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 6))
        frame.columnconfigure(1, weight=1)

        ttk.Label(frame, text="ID").grid(row=0, column=0, sticky="w", padx=(0, 8))
        self.id_var = tk.StringVar()
        self.id_entry = ttk.Entry(frame, textvariable=self.id_var)
        self.id_entry.grid(row=0, column=1, sticky="ew")
        self.id_entry.bind("<Return>", lambda _event: self._on_start())
        self.id_entry.focus_set()

        self.start_button = ttk.Button(frame, text="Start", command=self._on_start)
        self.start_button.grid(row=0, column=2, padx=(8, 0))
        self.stop_button = ttk.Button(
            frame, text="Stop", command=self._on_stop, state="disabled"
        )
        self.stop_button.grid(row=0, column=3, padx=(6, 0))

    def _build_status(self) -> None:
        frame = ttk.Frame(self.root, padding=(12, 0))
        frame.grid(row=1, column=0, sticky="ew")
        frame.columnconfigure(0, weight=1)

        self.phase_label = ttk.Label(
            frame, text="Idle", font=("TkDefaultFont", 16 + FONT_BUMP, "bold")
        )
        self.phase_label.grid(row=0, column=0, sticky="w")
        self.phase_detail = ttk.Label(frame, text="", foreground=_MUTED)
        self.phase_detail.grid(row=1, column=0, sticky="w")
        self.link_label = ttk.Label(frame, text="connecting ...", foreground=_MUTED)
        self.link_label.grid(row=2, column=0, sticky="w", pady=(2, 0))

    def _build_readings(self) -> None:
        frame = ttk.LabelFrame(self.root, text="Live readings", padding=10)
        frame.grid(row=2, column=0, sticky="nsew", padx=10, pady=8)
        frame.columnconfigure(1, weight=1)
        self.root.rowconfigure(2, weight=1)

        self.value_labels: dict[str, ttk.Label] = {}
        rows = (
            ("heart_rate_bpm", "Heart rate"),
            ("breathing_rate_bpm", "Breathing rate"),
            ("target_distance_cm", "Target distance"),
            ("motion_state", "Motion state"),
            ("body_movement", "Body movement"),
            ("presence", "Presence"),
        )
        for index, (name, label) in enumerate(rows):
            ttk.Label(frame, text=label).grid(row=index, column=0, sticky="w", pady=1)
            value = ttk.Label(frame, text="--", font=("TkFixedFont", 11 + FONT_BUMP))
            value.grid(row=index, column=1, sticky="w", padx=(12, 0))
            self.value_labels[name] = value

        ttk.Separator(frame, orient="horizontal").grid(
            row=len(rows), column=0, columnspan=2, sticky="ew", pady=8
        )
        self.verdict_label = ttk.Label(
            frame, text="waiting for data", font=("TkDefaultFont", 11 + FONT_BUMP, "bold")
        )
        self.verdict_label.grid(row=len(rows) + 1, column=0, columnspan=2, sticky="w")
        self.reason_label = ttk.Label(frame, text="", foreground=_MUTED, wraplength=480)
        self.reason_label.grid(row=len(rows) + 2, column=0, columnspan=2, sticky="w")

    def _build_footer(self) -> None:
        frame = ttk.Frame(self.root, padding=(12, 0, 12, 12))
        frame.grid(row=3, column=0, sticky="ew")
        frame.columnconfigure(0, weight=1)
        self.counter_label = ttk.Label(frame, text="", foreground=_MUTED)
        self.counter_label.grid(row=0, column=0, sticky="w")
        self.saved_label = ttk.Label(frame, text="", foreground=_OK, wraplength=520)
        self.saved_label.grid(row=1, column=0, sticky="w", pady=(4, 0))

    # -- actions -----------------------------------------------------------

    def _on_start(self) -> None:
        if self.core.phase != "idle":
            return
        subject_id = sanitise_id(self.id_var.get())
        if not subject_id:
            messagebox.showwarning("ID required", "Enter an ID for this session.")
            self.id_entry.focus_set()
            return
        if subject_id != self.id_var.get().strip():
            self.id_var.set(subject_id)  # show what the filename will actually be

        self.saved_label.config(text="")
        self.core.start(subject_id)
        self.id_entry.config(state="disabled")
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")

    def _on_stop(self) -> None:
        if self.core.phase == "idle":
            return
        warming = self.core.phase == "warmup"
        document = self.core.stop()
        self._reset_controls()
        if warming or not document["samples"]:
            self.saved_label.config(
                text="Stopped during warm-up -- nothing recorded, nothing saved.",
                foreground=_WARN,
            )
            return
        try:
            path = self._save(document)
        except OSError as exc:
            messagebox.showerror("Save failed", str(exc))
            return
        quality = document["quality"]
        self.saved_label.config(
            text=(
                f"Saved {quality['samples_valid']}/{quality['samples_total']} valid samples "
                f"-> {path}"
            ),
            foreground=_OK if quality["samples_valid"] else _WARN,
        )

    def _save(self, document: dict[str, Any]) -> Path:
        self.outdir.mkdir(parents=True, exist_ok=True)
        path = self.outdir / f"{document['id']}.json"
        if path.exists():
            overwrite = messagebox.askyesno(
                "File exists",
                f"{path.name} already exists.\n\n"
                "Overwrite it?  Choosing No saves alongside it with a timestamp.",
            )
            if not overwrite:
                stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
                path = self.outdir / f"{document['id']}-{stamp}.json"
        path.write_text(json.dumps(document, indent=2), encoding="utf-8")
        return path

    def _reset_controls(self) -> None:
        self.id_entry.config(state="normal")
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")

    def _on_close(self) -> None:
        if self.core.phase == "recording" and self.core.samples:
            if messagebox.askyesno("Recording in progress", "Save this session before closing?"):
                self._on_stop()
        self.root.destroy()

    # -- refresh -----------------------------------------------------------

    def _poll(self) -> None:
        self._render(self.core.snapshot())
        self.root.after(self.POLL_MS, self._poll)

    def _render(self, snap: dict[str, Any]) -> None:
        link = f"{snap['port']} @ {snap['baud']}" if snap["connected"] else snap["status_detail"]
        self.link_label.config(
            text=("● " if snap["connected"] else "○ ") + link,
            foreground=_OK if snap["connected"] else _BAD,
        )
        idle_and_ready = snap["phase"] == "idle" and snap["connected"]
        self.start_button.config(state="normal" if idle_and_ready else "disabled")

        phase = snap["phase"]
        if phase == "warmup":
            self.phase_label.config(text=f"Stabilising {snap['warmup_remaining']:4.1f}s", foreground=_WARN)
            self.phase_detail.config(
                text=(
                    f"reading but not recording -- letting the radar lock on "
                    f"({snap['warmup_frames']} frames seen)"
                )
            )
        elif phase == "recording":
            self.phase_label.config(text=f"Recording {snap['elapsed']:6.1f}s", foreground=_BAD)
            self.phase_detail.config(text=f"id: {snap['subject_id']} -- press Stop to save")
        else:
            self.phase_label.config(text="Idle", foreground=_MUTED)
            self.phase_detail.config(
                text="enter an ID and press Start"
                if snap["connected"]
                else "waiting for the radar"
            )

        for name, label in self.value_labels.items():
            entry = snap["fields"].get(name)
            if entry is None:
                label.config(text="--", foreground=_MUTED)
                continue
            value = entry["value"]
            shown = f"{value:g}" if isinstance(value, float) else str(value)
            if entry["label"]:
                shown = f"{shown} ({entry['label']})"
            stale = entry["age"] > self.core.gate.config.stale_after_s
            label.config(
                text=f"{shown:<18} {entry['age']:4.1f}s ago",
                foreground=_MUTED if stale else "",
            )

        verdict = snap["verdict"]
        if verdict["valid"]:
            self.verdict_label.config(text="✓ vitals look real", foreground=_OK)
            self.reason_label.config(text="occupancy, range, plausibility and freshness all pass")
        else:
            self.verdict_label.config(text="✗ vitals rejected", foreground=_BAD)
            self.reason_label.config(text=verdict["summary"])

        if phase == "recording":
            self.counter_label.config(
                text=(
                    f"{snap['samples']} samples  |  {snap['valid']} valid  |  "
                    f"{snap['rejected']} rejected  |  frames {snap['frames_ok']}"
                    + (f"  |  bad checksum {snap['bad_checksum']}" if snap["bad_checksum"] else "")
                )
            )
        else:
            self.counter_label.config(text=f"frames {snap['frames_ok']}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="r60a-recorder",
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", default="auto", help="serial device, or 'auto'")
    parser.add_argument("--baud", default="auto", help="baud rate, or 'auto'")
    parser.add_argument("--outdir", default="data", help="directory for <id>.json")
    parser.add_argument(
        "--warmup", type=float, default=DEFAULT_WARMUP_S,
        help="seconds to read without recording while the radar settles",
    )
    parser.add_argument(
        "--rate", type=float, default=DEFAULT_SAMPLE_HZ, help="samples per second"
    )
    parser.add_argument(
        "--probe-seconds", type=float, default=3.0, help="sample length per baud"
    )
    parser.add_argument(
        "--max-distance", type=float, default=GateConfig.max_distance_cm,
        help="reject vitals when the target is further away than this (cm)",
    )
    parser.add_argument("--hr-min", type=int, default=GateConfig.heart_min)
    parser.add_argument("--hr-max", type=int, default=GateConfig.heart_max)
    parser.add_argument("--br-min", type=int, default=GateConfig.breath_min)
    parser.add_argument("--br-max", type=int, default=GateConfig.breath_max)
    parser.add_argument(
        "--freeze-window", type=float, default=GateConfig.freeze_window_s,
        help="seconds of an unchanging vital before it counts as latched",
    )
    parser.add_argument(
        "--no-gate", action="store_true",
        help="record everything as valid (diagnostics; reasons are still logged)",
    )
    parser.add_argument(
        "--drop-invalid", action="store_true",
        help="omit rejected samples from the JSON instead of flagging them",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)

    if tk is None:
        print(
            "error: tkinter is not available in this Python.\n"
            "hint: sudo apt install python3-tk  (or use `python -m r60a.record` instead)",
            file=sys.stderr,
        )
        return 2

    config = GateConfig(
        enabled=not args.no_gate,
        max_distance_cm=args.max_distance,
        heart_min=args.hr_min,
        heart_max=args.hr_max,
        breath_min=args.br_min,
        breath_max=args.br_max,
        freeze_window_s=args.freeze_window,
    )
    core = RecorderCore(
        gate_config=config,
        warmup_s=args.warmup,
        sample_hz=args.rate,
        keep_invalid=not args.drop_invalid,
    )
    reader = RadarReader(core, args.port, args.baud, args.probe_seconds)
    reader.start()

    root = tk.Tk()
    RecorderApp(root, core, Path(args.outdir))
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
