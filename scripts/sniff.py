#!/usr/bin/env python3
"""Phase 1 discovery tool for a MicRadar R60A-series 60 GHz mmWave module.

Scans for a likely USB-serial adapter, samples raw bytes at several candidate
baud rates, and scores each sample by how well it resembles the MicRadar SIP
framing (0x53 0x59 ... 0x54 0x43).  Prints a hexdump of the best candidate.

Nothing here assumes the frame layout is correct -- the point is to find out.
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from typing import Iterable, Sequence

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - environment problem, not logic
    print("pyserial is required:  pip install pyserial", file=sys.stderr)
    raise SystemExit(2)

HEADER = b"\x53\x59"
TAIL = b"\x54\x43"

CANDIDATE_BAUDS: tuple[int, ...] = (115200, 256000, 9600)

# Substrings that show up in the description/hwid of the usual TTL adapters.
ADAPTER_HINTS: tuple[str, ...] = (
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

# Ports that are never a radar, however tempting the glob looks.
PORT_DENYLIST: tuple[str, ...] = (
    "bluetooth",
    "debug-console",
)


@dataclass
class FrameStat:
    """One header..tail candidate located inside a raw sample."""

    start: int
    end: int  # exclusive, just past the tail
    declared_len: int | None

    @property
    def total_len(self) -> int:
        return self.end - self.start


@dataclass
class BaudResult:
    baud: int
    raw: bytes = b""
    duration_s: float = 0.0
    frames: list[FrameStat] = field(default_factory=list)
    error: str | None = None

    @property
    def header_count(self) -> int:
        return count_occurrences(self.raw, HEADER)

    @property
    def tail_count(self) -> int:
        return count_occurrences(self.raw, TAIL)

    @property
    def score(self) -> int:
        """Well-formed header->tail pairs dominate; bare headers break ties."""
        return len(self.frames) * 100 + self.header_count

    @property
    def frame_rate_hz(self) -> float:
        if self.duration_s <= 0:
            return 0.0
        return len(self.frames) / self.duration_s

    @property
    def avg_frame_len(self) -> float:
        if not self.frames:
            return 0.0
        return sum(f.total_len for f in self.frames) / len(self.frames)

    @property
    def framing_efficiency(self) -> float:
        """Fraction of sampled bytes accounted for by complete frames."""
        if not self.raw:
            return 0.0
        return sum(f.total_len for f in self.frames) / len(self.raw)


def count_occurrences(haystack: bytes, needle: bytes) -> int:
    count = 0
    pos = haystack.find(needle)
    while pos != -1:
        count += 1
        pos = haystack.find(needle, pos + 1)
    return count


def find_frames(raw: bytes) -> list[FrameStat]:
    """Locate 0x53 0x59 ... 0x54 0x43 runs without trusting the length field.

    Two strategies are combined: if the 16-bit big-endian length at offset 4
    lands the tail exactly where it should be, take that (the strong signal).
    Otherwise fall back to the next tail that appears before the next header,
    which still tells us framing exists even if the layout differs.
    """
    frames: list[FrameStat] = []
    pos = raw.find(HEADER)
    while pos != -1:
        stat: FrameStat | None = None

        # Strategy A: trust the declared length and check the tail lands there.
        if pos + 6 <= len(raw):
            declared = int.from_bytes(raw[pos + 4 : pos + 6], "big")
            tail_at = pos + 6 + declared + 1  # +1 for the checksum byte
            if 0 <= declared <= 512 and tail_at + 2 <= len(raw):
                if raw[tail_at : tail_at + 2] == TAIL:
                    stat = FrameStat(pos, tail_at + 2, declared)

        # Strategy B: nearest tail before the next header.
        if stat is None:
            next_header = raw.find(HEADER, pos + 2)
            limit = next_header if next_header != -1 else len(raw)
            tail_at = raw.find(TAIL, pos + 2)
            if tail_at != -1 and tail_at < limit:
                stat = FrameStat(pos, tail_at + 2, None)

        if stat is not None:
            frames.append(stat)
            pos = raw.find(HEADER, stat.end)
        else:
            pos = raw.find(HEADER, pos + 2)

    return frames


def score_candidate_ports() -> list[tuple[str, str]]:
    """Return (device, description) for plausible adapters, best guess first."""
    ranked: list[tuple[int, str, str]] = []
    for port in list_ports.comports():
        blob = " ".join(
            str(x) for x in (port.device, port.description, port.hwid) if x
        ).lower()
        if any(bad in blob for bad in PORT_DENYLIST):
            continue
        hits = sum(1 for hint in ADAPTER_HINTS if hint in blob)
        if hits == 0 and not any(
            port.device.startswith(prefix)
            for prefix in ("/dev/ttyUSB", "/dev/ttyACM", "/dev/cu.usbserial")
        ):
            continue
        ranked.append((hits, port.device, port.description or "n/a"))
    ranked.sort(key=lambda row: (-row[0], row[1]))
    return [(device, desc) for _, device, desc in ranked]


def sample_baud(port: str, baud: int, seconds: float) -> BaudResult:
    result = BaudResult(baud=baud)
    chunks: list[bytes] = []
    try:
        with serial.Serial(port, baud, timeout=0.2) as ser:
            ser.reset_input_buffer()
            started = time.monotonic()
            deadline = started + seconds
            while time.monotonic() < deadline:
                chunk = ser.read(4096)
                if chunk:
                    chunks.append(chunk)
            result.duration_s = time.monotonic() - started
    except (serial.SerialException, OSError, ValueError) as exc:
        result.error = f"{type(exc).__name__}: {exc}"
        return result

    result.raw = b"".join(chunks)
    result.frames = find_frames(result.raw)
    return result


def hexdump(data: bytes, width: int = 16, limit: int | None = None) -> str:
    if limit is not None:
        data = data[:limit]
    lines: list[str] = []
    for offset in range(0, len(data), width):
        row = data[offset : offset + width]
        hex_part = " ".join(f"{b:02x}" for b in row).ljust(width * 3 - 1)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in row)
        lines.append(f"{offset:08x}  {hex_part}  |{ascii_part}|")
    return "\n".join(lines)


def summarize_frames(result: BaudResult, limit: int = 8) -> str:
    lines: list[str] = []
    for stat in result.frames[:limit]:
        body = result.raw[stat.start : stat.end]
        control = body[2] if len(body) > 2 else 0
        command = body[3] if len(body) > 3 else 0
        declared = "n/a" if stat.declared_len is None else str(stat.declared_len)
        lines.append(
            f"  @{stat.start:<6d} len={stat.total_len:<4d} declared={declared:<4s} "
            f"control=0x{control:02x} command=0x{command:02x}  {body.hex(' ')}"
        )
    if len(result.frames) > limit:
        lines.append(f"  ... {len(result.frames) - limit} more")
    return "\n".join(lines)


def report(results: Sequence[BaudResult], winner: BaudResult | None) -> None:
    print("\n=== baud scan ===")
    header = f"{'baud':>8}  {'bytes':>8}  {'frames':>7}  {'hdr':>5}  {'tail':>5}  {'Hz':>7}  {'avg len':>8}  note"
    print(header)
    print("-" * len(header))
    for res in results:
        note = res.error or ("<-- best" if res is winner else "")
        print(
            f"{res.baud:>8}  {len(res.raw):>8}  {len(res.frames):>7}  "
            f"{res.header_count:>5}  {res.tail_count:>5}  {res.frame_rate_hz:>7.2f}  "
            f"{res.avg_frame_len:>8.1f}  {note}"
        )

    if winner is None or not winner.raw:
        print("\nNo data captured at any baud rate.")
        return

    print(f"\n=== hexdump: first 500 bytes @ {winner.baud} baud ===")
    print(hexdump(winner.raw, limit=500))

    print(f"\n=== framing @ {winner.baud} baud ===")
    print(f"  sample duration    : {winner.duration_s:.2f} s")
    print(f"  bytes captured     : {len(winner.raw)}")
    print(f"  complete frames    : {len(winner.frames)}")
    print(f"  frame rate         : {winner.frame_rate_hz:.2f} Hz")
    print(f"  avg frame length   : {winner.avg_frame_len:.1f} bytes")
    print(f"  bytes inside frames: {winner.framing_efficiency * 100:.1f}%")
    lengths = sorted({f.total_len for f in winner.frames})
    if lengths:
        print(f"  distinct lengths   : {lengths}")
    pairs = sorted(
        {
            (winner.raw[f.start + 2], winner.raw[f.start + 3])
            for f in winner.frames
            if f.start + 4 <= len(winner.raw)
        }
    )
    if pairs:
        print("  (control, command) pairs seen:")
        for control, command in pairs:
            hits = sum(
                1
                for f in winner.frames
                if winner.raw[f.start + 2] == control
                and winner.raw[f.start + 3] == command
            )
            print(f"    0x{control:02x} 0x{command:02x}  x{hits}")

    if winner.frames:
        print("\n=== first frames ===")
        print(summarize_frames(winner))


def save_capture(path: str, data: bytes) -> None:
    with open(path, "wb") as handle:
        handle.write(data)
    print(f"\nRaw capture written to {path} ({len(data)} bytes)")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="auto", help="serial device, or 'auto'")
    parser.add_argument(
        "--baud",
        type=int,
        action="append",
        help="baud to try (repeatable); defaults to 115200, 256000, 9600",
    )
    parser.add_argument(
        "--seconds", type=float, default=3.0, help="sample duration per baud"
    )
    parser.add_argument("--save", help="write the winning raw capture to this file")
    parser.add_argument(
        "--list", action="store_true", help="list serial ports and exit"
    )
    return parser.parse_args(argv)


def print_all_ports() -> None:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found at all.")
        return
    print("All serial ports visible to the OS:")
    for port in ports:
        print(f"  {port.device:<32} {port.description or 'n/a'}  [{port.hwid or 'n/a'}]")


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)

    if args.list:
        print_all_ports()
        return 0

    if args.port == "auto":
        candidates = score_candidate_ports()
        if not candidates:
            print("No USB-serial adapter found.\n")
            print_all_ports()
            print(
                "\nExpected something like /dev/ttyUSB0 (Linux) or "
                "/dev/cu.usbserial-XXXX / /dev/cu.wchusbserialXXXX (macOS).\n"
                "Check that the CH340 is plugged in, and on macOS that a CH34x\n"
                "driver is loaded. Re-run with --port to override."
            )
            return 1
        port = candidates[0][0]
        if len(candidates) > 1:
            print("Multiple candidate adapters found:")
            for device, desc in candidates:
                print(f"  {device:<32} {desc}")
        print(f"Using port: {port}")
    else:
        port = args.port
        print(f"Using port: {port} (explicit)")

    bauds: Iterable[int] = tuple(args.baud) if args.baud else CANDIDATE_BAUDS

    results: list[BaudResult] = []
    for baud in bauds:
        print(f"Sampling {args.seconds:.1f}s @ {baud} baud ...", flush=True)
        res = sample_baud(port, baud, args.seconds)
        if res.error:
            print(f"  failed: {res.error}")
        else:
            print(
                f"  {len(res.raw)} bytes, {len(res.frames)} framed, "
                f"{res.header_count} headers"
            )
        results.append(res)

    scored = [r for r in results if r.error is None and r.raw]
    winner = max(scored, key=lambda r: r.score) if scored else None
    report(results, winner)

    if winner is None:
        print(
            "\nNothing decodable. If the module is powered, check TX->RXD wiring\n"
            "and that ground is shared between the radar and the adapter."
        )
        return 1

    if args.save:
        save_capture(args.save, winner.raw)

    if not winner.frames:
        print(
            "\nBytes arrived but no 53 59 ... 54 43 framing was found -- the baud\n"
            "may still be wrong, or this module uses a different protocol."
        )
        return 1

    print(f"\nDetected baud: {winner.baud}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
