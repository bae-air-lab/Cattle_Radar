"""Serial port and baud-rate detection for the R60A.

Shares the framing logic in :mod:`r60a.protocol`, so a baud rate only "wins"
if real, checksum-valid frames come out of it.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable

import serial
from serial.tools import list_ports

from .protocol import FrameParser

__all__ = [
    "CANDIDATE_BAUDS",
    "PortCandidate",
    "BaudProbe",
    "list_candidate_ports",
    "autodetect_port",
    "probe_baud",
    "autodetect_baud",
]

CANDIDATE_BAUDS: tuple[int, ...] = (115200, 256000, 9600)

_ADAPTER_HINTS: tuple[str, ...] = (
    "ch340",
    "ch341",
    "ch9102",
    "wch",
    "usbserial",
    "usb-serial",
    "ftdi",
    "cp210",
    "silicon labs",
    "prolific",
    "pl2303",
)

_PORT_DENYLIST: tuple[str, ...] = ("bluetooth", "debug-console")

_DEVICE_PREFIXES: tuple[str, ...] = (
    "/dev/ttyUSB",
    "/dev/ttyACM",
    "/dev/cu.usbserial",
    "/dev/cu.wchusbserial",
    "/dev/cu.SLAB",
)


class DiscoveryError(RuntimeError):
    """No usable port or baud rate could be found."""


@dataclass(frozen=True)
class PortCandidate:
    device: str
    description: str
    score: int


def list_candidate_ports() -> list[PortCandidate]:
    """Plausible USB-serial adapters, most likely first."""
    found: list[PortCandidate] = []
    for port in list_ports.comports():
        blob = " ".join(
            str(x) for x in (port.device, port.description, port.hwid) if x
        ).lower()
        if any(bad in blob for bad in _PORT_DENYLIST):
            continue
        score = sum(1 for hint in _ADAPTER_HINTS if hint in blob)
        if score == 0 and not port.device.startswith(_DEVICE_PREFIXES):
            continue
        found.append(PortCandidate(port.device, port.description or "n/a", score))
    found.sort(key=lambda c: (-c.score, c.device))
    return found


def autodetect_port() -> str:
    candidates = list_candidate_ports()
    if not candidates:
        visible = ", ".join(p.device for p in list_ports.comports()) or "none"
        raise DiscoveryError(
            f"No USB-serial adapter found (ports visible: {visible}). "
            "Check the CH340 is plugged in, or pass --port explicitly."
        )
    return candidates[0].device


@dataclass
class BaudProbe:
    baud: int
    frames_ok: int = 0
    frames_bad: int = 0
    bytes_read: int = 0
    error: str | None = None

    @property
    def score(self) -> int:
        return self.frames_ok * 10 - self.frames_bad


def probe_baud(port: str, baud: int, seconds: float = 3.0) -> BaudProbe:
    """Sample one baud rate and count the frames that parse cleanly."""
    probe = BaudProbe(baud=baud)
    parser = FrameParser()
    try:
        with serial.Serial(port, baud, timeout=0.2) as ser:
            ser.reset_input_buffer()
            deadline = time.monotonic() + seconds
            while time.monotonic() < deadline:
                chunk = ser.read(4096)
                if not chunk:
                    continue
                probe.bytes_read += len(chunk)
                parser.feed(chunk)
    except (serial.SerialException, OSError, ValueError) as exc:
        probe.error = f"{type(exc).__name__}: {exc}"
        return probe

    probe.frames_ok = parser.stats.frames_ok
    probe.frames_bad = parser.stats.frames_bad_checksum
    return probe


def autodetect_baud(
    port: str,
    bauds: tuple[int, ...] = CANDIDATE_BAUDS,
    seconds: float = 3.0,
    on_probe: Callable[[BaudProbe], None] | None = None,
) -> int:
    """Try each baud in turn, return the first that yields clean frames.

    Stops early on a decisive result so the common case costs one 3s sample
    rather than three.
    """
    probes: list[BaudProbe] = []
    for baud in bauds:
        probe = probe_baud(port, baud, seconds)
        probes.append(probe)
        if on_probe is not None:
            on_probe(probe)
        if probe.frames_ok >= 3 and probe.frames_bad == 0:
            return probe.baud

    usable = [p for p in probes if p.frames_ok > 0]
    if not usable:
        detail = ", ".join(
            f"{p.baud}:{p.error or f'{p.bytes_read}B/0 frames'}" for p in probes
        )
        raise DiscoveryError(
            f"No baud rate produced valid frames on {port} ({detail}). "
            "Check radar TX -> adapter RXD and a shared ground."
        )
    return max(usable, key=lambda p: p.score).baud
