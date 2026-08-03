"""Tools for the MicRadar R60A-series 60 GHz mmWave radar over a USB-TTL link."""

from __future__ import annotations

from .protocol import (
    MESSAGE_MAP,
    Frame,
    FrameParser,
    ParserStats,
    Reading,
    UnknownFrame,
    checksum,
    decode,
    encode_frame,
)

__version__ = "0.1.0"

__all__ = [
    "MESSAGE_MAP",
    "Frame",
    "FrameParser",
    "ParserStats",
    "Reading",
    "UnknownFrame",
    "checksum",
    "decode",
    "encode_frame",
    "__version__",
]
