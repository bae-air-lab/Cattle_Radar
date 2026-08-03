"""MicRadar R60A-series SIP-S serial protocol.

Frame layout, verified against a 950-byte / 86-frame capture taken at 115200
baud (see tests/fixtures/capture_115200.bin)::

    0x53 0x59 | control | command | len_hi | len_lo | data[len] | checksum | 0x54 0x43

``checksum`` is the sum of every byte from the leading 0x53 through the last
data byte, masked to 0xFF.  All 86 frames in the reference capture pass, and
the 16-bit big-endian length field landed the 0x54 0x43 tail at exactly the
predicted offset on every frame.

To teach the decoder a new message, add one entry to MESSAGE_MAP below.
Anything not in that dict is surfaced as an UnknownFrame rather than guessed at.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Final, Mapping

__all__ = [
    "HEADER",
    "TAIL",
    "MESSAGE_MAP",
    "ENUM_LABELS",
    "Frame",
    "Reading",
    "UnknownFrame",
    "FrameParser",
    "ParserStats",
    "checksum",
    "encode_frame",
    "decode",
]

HEADER: Final = b"\x53\x59"
TAIL: Final = b"\x54\x43"

#: Longest payload we will believe.  Guards against a false header whose
#: length field is garbage; real frames in this family are <= 32 bytes.
MAX_PAYLOAD: Final = 512

#: The 0x80/0x04 distance field arrives as a raw 16-bit integer.  Observed
#: values sit steady at 59 for a person about half a metre away, so the raw
#: unit is centimetres.  Flip this to False if your datasheet says millimetres.
DISTANCE_RAW_IS_CM: Final = True

Value = int | float | str
Decoder = Callable[[bytes], dict[str, Value]]


def _u8(payload: bytes) -> int:
    return payload[0]


def _u16_be(payload: bytes) -> int:
    return int.from_bytes(payload[:2], "big")


def _sign_magnitude(raw: int) -> int:
    """Decode a 16-bit sign-magnitude coordinate.

    The R60A sets bit 15 as a negative flag rather than using two's
    complement: 0x801f is -31, not -32737.
    """
    return -(raw & 0x7FFF) if raw & 0x8000 else raw


def _presence(payload: bytes) -> dict[str, Value]:
    return {"presence": _u8(payload)}


def _motion_state(payload: bytes) -> dict[str, Value]:
    return {"motion_state": _u8(payload)}


def _body_movement(payload: bytes) -> dict[str, Value]:
    return {"body_movement": _u8(payload)}


def _distance(payload: bytes) -> dict[str, Value]:
    raw = _u16_be(payload)
    return {"target_distance_cm": float(raw) if DISTANCE_RAW_IS_CM else raw / 10.0}


def _coordinates(payload: bytes) -> dict[str, Value]:
    x = _sign_magnitude(int.from_bytes(payload[0:2], "big"))
    y = _sign_magnitude(int.from_bytes(payload[2:4], "big"))
    z = _sign_magnitude(int.from_bytes(payload[4:6], "big"))
    return {"target_x_cm": x, "target_y_cm": y, "target_z_cm": z}


def _breathing_rate(payload: bytes) -> dict[str, Value]:
    return {"breathing_rate_bpm": _u8(payload)}


def _heart_rate(payload: bytes) -> dict[str, Value]:
    return {"heart_rate_bpm": _u8(payload)}


def _heartbeat(payload: bytes) -> dict[str, Value]:
    return {"heartbeat": _u8(payload)}


@dataclass(frozen=True)
class MessageSpec:
    """How to turn one (control, command) payload into named readings."""

    name: str
    decode: Decoder
    unit: str = ""
    expected_len: int | None = None
    verified: bool = True
    """False means the mapping comes from the datasheet/family conventions but
    was not observed in the Phase 1 capture."""


# ---------------------------------------------------------------------------
# THE map.  Extend this against the datasheet; nothing else needs to change.
# ---------------------------------------------------------------------------
MESSAGE_MAP: Final[Mapping[tuple[int, int], MessageSpec]] = {
    (0x01, 0x01): MessageSpec("heartbeat", _heartbeat, "", 1),
    (0x80, 0x01): MessageSpec(
        "presence", _presence, "0=absent 1=present", 1, verified=False
    ),
    (0x80, 0x02): MessageSpec("motion_state", _motion_state, "0=none 1=still 2=active", 1),
    (0x80, 0x03): MessageSpec("body_movement", _body_movement, "0-100", 1),
    (0x80, 0x04): MessageSpec("target_distance", _distance, "cm", 2),
    (0x80, 0x05): MessageSpec("target_position", _coordinates, "cm", 6),
    (0x81, 0x02): MessageSpec("breathing_rate", _breathing_rate, "breaths/min", 1),
    (0x85, 0x02): MessageSpec("heart_rate", _heart_rate, "bpm", 1),
}

#: Display-only labels for the enumerated fields.  The CSV keeps the raw
#: integer so downstream analysis stays numeric.
ENUM_LABELS: Final[Mapping[str, Mapping[int, str]]] = {
    "presence": {0: "absent", 1: "present"},
    "motion_state": {0: "none", 1: "still", 2: "active"},
}


def checksum(data: bytes) -> int:
    """Sum of every byte from the header through the last data byte."""
    return sum(data) & 0xFF


def encode_frame(control: int, command: int, payload: bytes = b"") -> bytes:
    """Build a well-formed frame.  Used by the tests, and ready for TX once
    the adapter's TXD line is wired to the radar's RX."""
    body = bytes(HEADER) + bytes((control, command)) + len(payload).to_bytes(2, "big") + payload
    return body + bytes((checksum(body),)) + bytes(TAIL)


@dataclass(frozen=True)
class Frame:
    """One framed message lifted off the wire, checksum verified but not decoded."""

    control: int
    command: int
    payload: bytes
    timestamp: float
    valid_checksum: bool
    raw: bytes

    @property
    def key(self) -> tuple[int, int]:
        return (self.control, self.command)

    @property
    def raw_hex(self) -> str:
        return self.raw.hex(" ")

    def __str__(self) -> str:
        flag = "" if self.valid_checksum else " !checksum"
        return f"<Frame 0x{self.control:02x}/0x{self.command:02x} {self.payload.hex(' ')}{flag}>"


@dataclass(frozen=True)
class Reading:
    """One decoded field from one frame -- a single CSV row."""

    timestamp: float
    control: int
    command: int
    message: str
    field_name: str
    value: Value
    unit: str
    raw_hex: str
    valid_checksum: bool = True

    @property
    def display_value(self) -> str:
        labels = ENUM_LABELS.get(self.field_name)
        if labels is not None and isinstance(self.value, int):
            return f"{self.value} ({labels.get(self.value, '?')})"
        if isinstance(self.value, float):
            return f"{self.value:g}"
        return str(self.value)


@dataclass(frozen=True)
class UnknownFrame:
    """A well-framed message whose (control, command) pair is not in MESSAGE_MAP."""

    timestamp: float
    control: int
    command: int
    payload_hex: str
    raw_hex: str
    valid_checksum: bool = True

    @property
    def key(self) -> tuple[int, int]:
        return (self.control, self.command)


def decode(frame: Frame) -> list[Reading] | UnknownFrame:
    """Turn a Frame into named readings, or report it as unknown.

    Never guesses: a pair absent from MESSAGE_MAP comes back as an
    UnknownFrame carrying the raw hex.  A payload whose length contradicts the
    spec is also treated as unknown rather than decoded from truncated bytes.
    """
    spec = MESSAGE_MAP.get(frame.key)
    if spec is None:
        return UnknownFrame(
            timestamp=frame.timestamp,
            control=frame.control,
            command=frame.command,
            payload_hex=frame.payload.hex(" "),
            raw_hex=frame.raw_hex,
            valid_checksum=frame.valid_checksum,
        )

    if spec.expected_len is not None and len(frame.payload) < spec.expected_len:
        return UnknownFrame(
            timestamp=frame.timestamp,
            control=frame.control,
            command=frame.command,
            payload_hex=frame.payload.hex(" "),
            raw_hex=frame.raw_hex,
            valid_checksum=frame.valid_checksum,
        )

    fields = spec.decode(frame.payload)
    return [
        Reading(
            timestamp=frame.timestamp,
            control=frame.control,
            command=frame.command,
            message=spec.name,
            field_name=name,
            value=value,
            unit=spec.unit,
            raw_hex=frame.raw_hex,
            valid_checksum=frame.valid_checksum,
        )
        for name, value in fields.items()
    ]


class _NeedMore:
    """Sentinel: the buffer holds too little to decide anything yet."""

    __slots__ = ()


_NEED_MORE: Final = _NeedMore()


@dataclass
class ParserStats:
    """Counters covering everything the parser saw, including what it threw away."""

    frames_ok: int = 0
    frames_bad_checksum: int = 0
    bytes_dropped: int = 0
    resyncs: int = 0
    bytes_fed: int = 0

    @property
    def frames_total(self) -> int:
        return self.frames_ok + self.frames_bad_checksum


class FrameParser:
    """Streaming, non-blocking parser for the SIP-S byte stream.

    Feed it whatever ``serial.read()`` returns -- partial frames, several
    frames at once, or pure garbage.  It buffers across calls, resynchronises
    by scanning forward for the next header, and returns only complete frames.
    Bad-checksum frames are returned too (flagged and counted), never dropped
    silently.
    """

    def __init__(self, max_payload: int = MAX_PAYLOAD, max_buffer: int = 8192) -> None:
        self._buffer = bytearray()
        self._max_payload = max_payload
        self._max_buffer = max_buffer
        self.stats = ParserStats()

    @property
    def buffered(self) -> int:
        return len(self._buffer)

    def reset(self) -> None:
        """Drop buffered bytes.  Call after a reconnect so a half-frame from
        the previous connection cannot merge with the new stream."""
        self.stats.bytes_dropped += len(self._buffer)
        self._buffer.clear()

    def feed(self, chunk: bytes, timestamp: float | None = None) -> list[Frame]:
        """Add bytes and return every frame that completed as a result."""
        if timestamp is None:
            timestamp = time.time()
        self.stats.bytes_fed += len(chunk)
        self._buffer.extend(chunk)

        frames: list[Frame] = []
        while True:
            result = self._next_frame(timestamp)
            if isinstance(result, _NeedMore):
                break
            if result is not None:
                frames.append(result)
            # result is None: we resynchronised past a false header, so go
            # round again -- a real frame may follow in the same buffer.

        self._trim()
        return frames

    def _next_frame(self, timestamp: float) -> Frame | _NeedMore | None:
        """Return the next frame, ``None`` if we resynced and should retry, or
        the _NEED_MORE sentinel if the buffer cannot yet decide."""
        buf = self._buffer

        start = buf.find(HEADER)
        if start == -1:
            # No header at all.  Keep a trailing 0x53 in case the 0x59 is in
            # the next read; discard everything else.
            keep = 1 if buf[-1:] == HEADER[:1] else 0
            discard = len(buf) - keep
            if discard > 0:
                self.stats.bytes_dropped += discard
                self.stats.resyncs += 1
                del buf[:discard]
            return _NEED_MORE

        if start > 0:
            self.stats.bytes_dropped += start
            self.stats.resyncs += 1
            del buf[:start]

        if len(buf) < 6:
            return _NEED_MORE  # need control, command and the length field

        declared = int.from_bytes(buf[4:6], "big")
        if declared > self._max_payload:
            return self._skip_false_header()

        total = 6 + declared + 1 + 2  # payload + checksum + tail
        if len(buf) < total:
            return _NEED_MORE  # frame still arriving

        if bytes(buf[total - 2 : total]) != TAIL:
            return self._skip_false_header()

        raw = bytes(buf[:total])
        payload = raw[6 : 6 + declared]
        expected = checksum(raw[: 6 + declared])
        actual = raw[6 + declared]
        valid = expected == actual

        if valid:
            self.stats.frames_ok += 1
        else:
            self.stats.frames_bad_checksum += 1

        del buf[:total]
        return Frame(
            control=raw[2],
            command=raw[3],
            payload=payload,
            timestamp=timestamp,
            valid_checksum=valid,
            raw=raw,
        )

    def _skip_false_header(self) -> None:
        """Advance one byte past a header that did not pan out.

        One byte, not two, so an overlapping sequence such as 53 53 59 ...
        still resynchronises onto the real header.
        """
        self.stats.bytes_dropped += 1
        self.stats.resyncs += 1
        del self._buffer[:1]
        return None

    def _trim(self) -> None:
        """Bound memory if the stream is garbage that happens to start with 0x53."""
        excess = len(self._buffer) - self._max_buffer
        if excess > 0:
            self.stats.bytes_dropped += excess
            del self._buffer[:excess]


@dataclass
class DecodeStats:
    """Aggregate counters for a decode session, kept alongside ParserStats."""

    readings: int = 0
    unknown_frames: int = 0
    unknown_pairs: dict[tuple[int, int], int] = field(default_factory=dict)

    def note_unknown(self, frame: UnknownFrame) -> None:
        self.unknown_frames += 1
        self.unknown_pairs[frame.key] = self.unknown_pairs.get(frame.key, 0) + 1
