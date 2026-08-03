# R60A mmWave radar toolkit

Read, decode, log and visualise a **MicRadar R60A-series 60 GHz mmWave radar**
over a USB-to-TTL serial adapter.

The R60A is a contactless vital-signs radar. It sits on a desk or wall, points
at a person, and continuously reports whether someone is there, where they are
in two dimensions, how much they are moving, and — the interesting part — their
**breathing rate and heart rate**, derived from sub-millimetre chest-wall
displacement. No camera, no contact, works through clothing and in the dark.

This repository contains the host-side software: the serial protocol decoder,
a capture tool that writes tidy CSV, and a live web dashboard.

> **You do not need the hardware to run any of this.** A real 20-second
> recording is checked into `data/sample_session.csv`, and the dashboard can
> replay it on a loop. See [Running without hardware](#running-without-hardware).

<!-- Add a screenshot here once you have one:
     ![dashboard](docs/dashboard.png) -->

---

## Table of contents

- [Quick start](#quick-start)
- [Running without hardware](#running-without-hardware)
- [What the radar actually reports](#what-the-radar-actually-reports)
- [The serial protocol](#the-serial-protocol)
- [How this protocol was established](#how-this-protocol-was-established)
- [Architecture](#architecture)
- [The tools in detail](#the-tools-in-detail)
- [CSV format](#csv-format)
- [Extending the decoder](#extending-the-decoder)
- [Testing](#testing)
- [Hardware setup](#hardware-setup)
- [Troubleshooting](#troubleshooting)
- [Design decisions and open questions](#design-decisions-and-open-questions)

---

## Quick start

Requires **Python 3.11+**. The only runtime dependency is `pyserial`.

```bash
git clone https://github.com/bae-air-lab/r60a-radar.git
cd r60a-radar
python3 -m pip install -e .
```

Then, with a radar plugged in:

```bash
# Live browser dashboard, opens http://127.0.0.1:8420
python3 -m r60a.visualize --port auto --baud auto

# Log 60 seconds to CSV with a live terminal readout
python3 -m r60a.record --port auto --baud auto --out data/session.csv --duration 60
```

`--port auto` scans for a USB-serial adapter; `--baud auto` samples each
candidate rate and keeps the one that yields checksum-valid frames. Both accept
explicit values (`--port /dev/ttyUSB0 --baud 115200`) when you already know.

After `pip install -e .` the console scripts `r60a-record` and `r60a-visualize`
work without the `python3 -m` prefix. Everything also runs straight from a
clone with no install at all, as long as `pyserial` is importable.

---

## Running without hardware

This is the path to take if you are picking up the software side and the radar
is on someone else's desk.

```bash
python3 -m pip install pyserial          # the only dependency
python3 -m r60a.visualize --replay data/sample_session.csv --speed 2
```

That opens the full dashboard on <http://127.0.0.1:8420> and drives it from a
real 20-second recording of a person sitting in front of the sensor — genuine
heart rate (71–79 bpm), breathing rate (14–19 breaths/min), position and
movement data, played at twice real time. The replay loops, so it keeps running. Everything in the UI behaves exactly as it
does live, because replay feeds the *same* parser the same bytes: the CSV
stores each frame's complete raw hex, and the replayer pushes those bytes back
through `FrameParser` rather than shortcutting to the decoded values.

The test suite also runs with no hardware and is the fastest way to understand
the protocol:

```bash
python3 -m pip install pytest
python3 -m pytest -q          # 35 tests, ~1 second
```

`tests/fixtures/capture_115200.bin` is 950 bytes of raw radar output — the
original capture the protocol was reverse-engineered from. `tests/test_protocol.py`
parses it and asserts on the result, so it doubles as executable documentation.

To poke at the protocol directly:

```python
from r60a.protocol import FrameParser, decode

parser = FrameParser()
for frame in parser.feed(open("tests/fixtures/capture_115200.bin", "rb").read()):
    print(frame, decode(frame))
```

---

## What the radar actually reports

The module transmits autonomously — it needs no polling, no commands, no
handshake. Power it up and it starts talking. Each measurement arrives as its
own small frame, at roughly 3 frames per second in total.

| Reading | Units | Notes |
|---|---|---|
| Presence / occupancy | 0 or 1 | reported on change only; see [open questions](#design-decisions-and-open-questions) |
| Motion state | 0 none, 1 still, 2 active | "still" means present but not moving much |
| Body movement magnitude | 0–100 | spikes when the subject shifts or gestures |
| Target distance | cm | straight-line range to the tracked person |
| Target position | cm | x (lateral), y (range), z — one target at a time |
| Breathing rate | breaths/min | typically 12–25 for a resting adult |
| Heart rate | bpm | typically 55–90 for a resting adult |
| Heartbeat | — | protocol keepalive, unrelated to the cardiac reading |

The vitals need a few seconds of a reasonably still subject to converge, and
they degrade with distance and with the subject facing away. Treat them as a
research-grade signal, not a medical device.

---

## The serial protocol

MicRadar calls it **SIP-S**. Every frame:

```
 0x53 0x59 | control | command | len_hi | len_lo | data[len] | checksum | 0x54 0x43
 └───────┘   └─────┘   └─────┘   └───────────┘   └─────────┘   └──────┘   └───────┘
  header      what        which     payload         payload     sum &      tail
  "SY"        subsystem   message   length, BE      bytes        0xFF      "TC"
```

- **Header** is ASCII `SY`, **tail** is ASCII `TC` — visible in a hexdump, which
  is what makes the framing easy to spot.
- **Length** is 16-bit big-endian and counts only the payload.
- **Checksum** is the sum of every byte from the leading `0x53` through the last
  payload byte, masked to `0xFF`. It does *not* include the tail.
- **(control, command)** together identify the message. Control is roughly a
  subsystem — `0x80` human presence, `0x81` respiration, `0x85` heart rate.

A concrete frame, heart rate = 71 bpm:

```
53 59 85 02 00 01 47 7b 54 43
─────  ──  ──  ───── ──  ── ─────
 SY    │   │     │    │   │   TC
       │   │     │    │   └── checksum 0x7b
       │   │     │    └────── payload: 0x47 = 71 bpm
       │   │     └─────────── length: 1 byte
       │   └───────────────── command 0x02
       └───────────────────── control 0x85 (heart rate)
```

Verify it yourself: `0x53+0x59+0x85+0x02+0x00+0x01+0x47 = 0x17b`, and
`0x17b & 0xFF = 0x7b`. ✓

### Message map

| control | command | Decoded field(s) | Unit |
|---|---|---|---|
| `0x01` | `0x01` | `heartbeat` | — |
| `0x80` | `0x01` | `presence` | 0 absent / 1 present |
| `0x80` | `0x02` | `motion_state` | 0 none / 1 still / 2 active |
| `0x80` | `0x03` | `body_movement` | 0–100 |
| `0x80` | `0x04` | `target_distance_cm` | cm |
| `0x80` | `0x05` | `target_x_cm`, `target_y_cm`, `target_z_cm` | cm |
| `0x81` | `0x02` | `breathing_rate_bpm` | breaths/min |
| `0x85` | `0x02` | `heart_rate_bpm` | bpm |

This table lives in exactly one place in the code — `MESSAGE_MAP` at the top of
[`r60a/protocol.py`](r60a/protocol.py) — and everything else derives from it.

**Coordinates are sign-magnitude, not two's complement.** Bit 15 is a negative
flag, so `0x801f` is −31 cm, not −32737. This is the single most likely thing
to get wrong when writing a decoder for this family.

Serial line settings: **115200 baud, 8N1, no flow control.**

---

## How this protocol was established

Worth reading before you trust any of the above, because it tells you which
parts are solid and which are inference.

The vendor documentation for this module is thin and partly inconsistent, so
the framing was confirmed empirically:

1. **`scripts/sniff.py`** sampled the port at 115200, 256000 and 9600 baud for
   three seconds each, scoring each sample by how many `53 59 … 54 43` runs it
   contained. 115200 produced 86 well-formed frames across 30 seconds with
   **100% of received bytes accounted for by complete frames**; the other rates
   produced pure noise.
2. The **length field** was validated by checking that the declared payload
   length placed the `54 43` tail at exactly the predicted offset. It did, on
   every one of the 86 frames.
3. The **checksum rule** was validated by recomputing it for all 86 frames:
   **86 passed, 0 failed.**
4. The **field semantics** were inferred from plausibility and then corroborated
   by live behaviour. `0x81/0x02` carried values of 21–22 while a person sat in
   front of the sensor; `0x85/0x02` carried 68–73. Those are exactly a resting
   adult's breathing and heart rates, and no other interpretation of a
   single-byte payload produces two such physiologically coherent numbers
   simultaneously.
5. The **sign-magnitude coordinate encoding** was confirmed by observing the
   same magnitude arrive with both signs — `0x801f` and `0x001f`, i.e. ∓31 cm —
   which two's complement cannot produce.

That capture is preserved as `tests/fixtures/capture_115200.bin` and is
re-parsed on every test run, so any regression in the parser fails the build
against real bytes rather than synthetic ones.

**What is still unconfirmed** is listed in
[open questions](#design-decisions-and-open-questions). The decoder never
guesses: an unrecognised `(control, command)` pair is surfaced as an
`UnknownFrame` carrying its raw hex, counted in the session summary, and shown
in the dashboard as *unidentified*.

---

## Architecture

```
                    ┌──────────────────┐
   radar TX ───────▶│  pyserial        │   arbitrary chunk boundaries
                    └────────┬─────────┘
                             ▼
                    ┌──────────────────┐   buffers across reads, resyncs
                    │  FrameParser     │   after garbage, verifies checksums
                    └────────┬─────────┘
                             ▼  Frame(control, command, payload, ts, valid)
                    ┌──────────────────┐   MESSAGE_MAP lookup; unknown pairs
                    │  decode()        │   pass through untouched
                    └────────┬─────────┘
                             ▼  list[Reading] | UnknownFrame
              ┌──────────────┴──────────────┐
              ▼                             ▼
      ┌───────────────┐            ┌──────────────────┐
      │  record.py    │            │  visualize.py    │
      │  CSV + TUI    │            │  HTTP + SSE      │──▶ browser dashboard
      └───────────────┘            └──────────────────┘
```

| File | Role |
|---|---|
| [`r60a/protocol.py`](r60a/protocol.py) | Framing, checksums, `MESSAGE_MAP`, the streaming `FrameParser`, `decode()`. Pure logic, no I/O — this is the file to read first. |
| [`r60a/discovery.py`](r60a/discovery.py) | Port scanning and baud probing. A baud rate only wins if it produces checksum-valid frames. |
| [`r60a/record.py`](r60a/record.py) | Capture CLI: CSV writer, live terminal display, reconnect-with-backoff, session summary. |
| [`r60a/visualize.py`](r60a/visualize.py) | Standard-library HTTP server streaming state to the browser over server-sent events. Also the CSV replay engine. |
| [`r60a/static/dashboard.html`](r60a/static/dashboard.html) | The whole dashboard — HTML, CSS and hand-rolled SVG charts in one self-contained file. No build step, no CDN, no framework. |
| [`scripts/sniff.py`](scripts/sniff.py) | Standalone discovery tool, deliberately dependency-free and independent of the package. |

### The parser is the interesting part

Serial data does not arrive in frame-sized pieces. `serial.read()` returns
whatever happens to be in the OS buffer: half a frame, three frames, a frame
plus the first two bytes of the next one, or nothing at all.
[`FrameParser`](r60a/protocol.py) is therefore a streaming state machine:

- **Never assume `one read() == one frame`.** It buffers across calls.
- **Resynchronise after garbage** by scanning forward for the next header. It
  advances one byte at a time, not two, so an overlapping run like
  `53 53 59 …` still locks onto the real header.
- **Reject false headers.** A `53 59` whose length field is absurd, or whose
  tail does not land where the length says it should, is treated as noise and
  skipped rather than swallowing the real frame behind it.
- **Never block, never drop silently.** It returns whatever completed and keeps
  the rest. Bad-checksum frames are returned *flagged*, not discarded, and
  every dropped byte is counted.
- **Bounded memory.** A stream of garbage that happens to contain `0x53` cannot
  grow the buffer without limit.

The test suite asserts that feeding the same 950-byte capture in chunks of 1,
3, 7, 64 and 512 bytes produces byte-identical frames. That property is what
makes the parser trustworthy against a real UART.

---

## The tools in detail

### `r60a.record` — capture to CSV

```bash
python3 -m r60a.record --port auto --baud auto --out data/session.csv --duration 60
```

Writes one CSV row per decoded field, shows a live terminal display of the
latest value of every field plus frames/sec and the bad-checksum count, and
reconnects automatically with exponential backoff if the adapter is unplugged.
Ctrl-C flushes the CSV and prints a summary.

| Flag | Default | Meaning |
|---|---|---|
| `--port` | `auto` | serial device, or `auto` to scan |
| `--baud` | `auto` | baud rate, or `auto` to probe 115200/256000/9600 |
| `--out` | `data/session.csv` | CSV path; `none` disables writing |
| `--duration` | `60` | seconds to record; `0` or negative runs until Ctrl-C |
| `--keep-bad` | off | also decode and log frames that failed their checksum |
| `--show-unknown` | off | print unrecognised frames as they arrive |
| `--no-live` | off | disable the live display (for logging to a file) |
| `--probe-seconds` | `3.0` | sample length per baud during `--baud auto` |

### `r60a.visualize` — live dashboard

```bash
python3 -m r60a.visualize --port auto --baud auto          # live
python3 -m r60a.visualize --replay data/sample_session.csv # no hardware
```

Serves <http://127.0.0.1:8420>. The page shows heart rate as the lead figure
alongside occupancy, breathing rate, distance and a body-movement meter; four
trend charts; a plan-view position plot with the target's track and 50 cm range
rings; a message inventory of every `(control, command)` pair seen; and a raw
frame log with the exact bytes. Charts have hover crosshairs and tooltips,
there is a table view for the text equivalent of every chart, and it follows
the system light/dark theme with a manual toggle.

| Flag | Default | Meaning |
|---|---|---|
| `--replay FILE` | — | drive the dashboard from a recorded CSV instead of the radar |
| `--speed` | `1.0` | replay speed multiplier |
| `--out FILE` | — | also log decoded readings to CSV while the dashboard runs |
| `--http-port` | `8420` | HTTP port |
| `--host` | `127.0.0.1` | bind address — change with care, the server has no auth |
| `--no-browser` | off | do not open a browser window |

Implementation note: no web framework. `http.server` plus server-sent events,
about 80 lines of transport. The browser holds an `EventSource` on `/events`
and receives a complete state snapshot five times a second;
`GET /api/state` returns the same snapshot as plain JSON, which is handy for
scripting against.

### `scripts/sniff.py` — protocol discovery

```bash
python3 scripts/sniff.py --seconds 5 --save capture.bin
python3 scripts/sniff.py --list          # just list serial ports
```

Run this first against unfamiliar hardware. It reports the winning baud rate, a
hexdump, frame rate, frame lengths, and every `(control, command)` pair it saw
with counts. It is intentionally standalone — no imports from `r60a` — so it
still works when the protocol assumptions in the package are wrong.

---

## CSV format

One row per decoded **field**, so a single position frame produces three rows
that share a timestamp and raw hex:

```csv
unix_ts,iso_time,control,command,field_name,value,raw_hex
1785780689.351,2026-08-03T14:11:29.351-04:00,0x80,0x05,target_x_cm,31,53 59 80 05 00 06 00 1f 00 32 00 00 88 54 43
1785780689.351,2026-08-03T14:11:29.351-04:00,0x80,0x05,target_y_cm,50,53 59 80 05 00 06 00 1f 00 32 00 00 88 54 43
1785780689.351,2026-08-03T14:11:29.351-04:00,0x80,0x05,target_z_cm,0,53 59 80 05 00 06 00 1f 00 32 00 00 88 54 43
```

Long format rather than wide, because the radar's fields arrive at different
rates and a wide table would be mostly holes. Pivot when you need to:

```python
import pandas as pd
df = pd.read_csv("data/sample_session.csv")
wide = df.pivot_table(index="unix_ts", columns="field_name", values="value")
hr = df[df.field_name == "heart_rate_bpm"]
```

Because `raw_hex` carries the complete frame, a CSV is a lossless record of the
session — you can re-decode it after fixing a decoder bug, which is exactly
what `--replay` does.

---

## Extending the decoder

Adding a message is one entry in `MESSAGE_MAP` and, if the payload shape is new,
a small decode function. Both live at the top of
[`r60a/protocol.py`](r60a/protocol.py):

```python
def _sleep_state(payload: bytes) -> dict[str, Value]:
    return {"sleep_state": payload[0]}

MESSAGE_MAP = {
    ...
    (0x84, 0x01): MessageSpec("sleep_state", _sleep_state, "0=awake 1=light 2=deep", 1),
}
```

That is the whole change. The CSV writer, the live display, the dashboard, the
message inventory and the session summary all read from `MESSAGE_MAP` and pick
it up automatically.

A `MessageSpec` carries a human name, the decode function, a unit string for
display, the expected payload length, and a `verified` flag — set it `False`
for anything taken from the datasheet but not yet seen on real hardware, and
the dashboard will label it *unconfirmed*.

To find candidates for the map, run `--show-unknown` and watch what the radar
emits that you cannot yet name:

```bash
python3 -m r60a.record --show-unknown --duration 120 --out none
```

Guard rails worth preserving if you refactor:

- If `expected_len` does not match the payload, the frame is reported as
  unknown rather than decoded from truncated bytes.
- Decoders receive only the payload, never the framing bytes.
- Adding to the map should never require touching the parser.

---

## Testing

```bash
python3 -m pytest -q
```

35 tests, no hardware required, about a second. Coverage:

| Area | Cases |
|---|---|
| Framing | clean frame; split across two reads; split at *every* byte boundary; delivered one byte at a time; two back-to-back frames in one chunk; three frames with garbage between |
| Resync | leading garbage; false header with an absurd length; overlapping `53 53 59` header bytes |
| Integrity | corrupted checksum byte; corrupted payload with an intact checksum; bad frames kept and counted, not dropped |
| Decoding | each known message; sign-magnitude coordinates; unknown pairs surfaced not guessed; short payload on a known pair rejected |
| Robustness | buffer bounded against endless garbage; `reset()` clears partial frames |
| Regression | the real 950-byte capture parses to 86 frames, 0 bad checksums, 0 dropped bytes, and yields identical output at five different chunk sizes |

Hand-built byte strings in the tests are real frames copied out of the capture,
annotated with what they mean, so the test file is a readable protocol
reference in its own right.

---

## Hardware setup

### Wiring

| Radar (R60A) | USB-TTL adapter (CH340) | Notes |
|---|---|---|
| 5V | 5V | the module is 5 V powered |
| GND | GND | **must** be common, or the UART floats and you get garbage |
| TX | RXD | the only data line in use |
| RX | *(not connected)* | leave open until you need to send commands |

This is a **receive-only** setup. The adapter's TXD is deliberately not wired,
so the host can never transmit. The module reports autonomously, so nothing is
lost by this — but configuration commands (changing report rate, setting
detection thresholds) are unavailable until that line is connected.

`protocol.encode_frame(control, command, payload)` already builds correctly
framed and checksummed outbound frames, and is exercised by the tests, so the
transmit path is ready the day the wire goes in.

### Ports by platform

| Platform | Device name |
|---|---|
| Linux | `/dev/ttyUSB0` |
| macOS | `/dev/cu.usbserial-XXXX` or `/dev/cu.wchusbserialXXXX` |
| Windows | `COMx` — untested, but pyserial and the parser are platform-agnostic |

On Linux you may need to be in the `dialout` group:
`sudo usermod -a -G dialout $USER`, then log out and back in. On macOS a CH34x
driver must be present; recent releases ship one.

---

## Troubleshooting

**"No USB-serial adapter found."** Run `python3 scripts/sniff.py --list` to see
every port the OS exposes. The scanner deliberately skips Bluetooth and debug
consoles. If your adapter is listed but not selected, pass it explicitly with
`--port`.

**Bytes arrive but no frames are found.** Almost always the baud rate. Run
`scripts/sniff.py`, which reports how many `53 59 … 54 43` runs it found at
each rate. If every rate scores zero, suspect the wiring — a missing common
ground produces exactly this symptom.

**Frames parse but the checksum count climbs.** Electrical noise, a long or
unshielded cable, or a shared ground that is not actually shared. The frames
are still returned and flagged, so `--keep-bad` lets you inspect them.

**Vitals read zero or implausible values.** The subject needs to be within
about 1.5 m, roughly facing the sensor, and reasonably still for several
seconds. Motion swamps the cardiac signal — that is physics, not a bug.

**Unknown pairs in the summary.** Expected and by design. Capture them with
`--show-unknown` and add them to `MESSAGE_MAP` once you know what they mean.

---

## Design decisions and open questions

**Distance is centimetres, not millimetres.** The vendor material suggests mm,
but the raw value sits steady at 59 for a person about half a metre away, and
59 mm would place them 6 cm from the antenna. Flip `DISTANCE_RAW_IS_CM` in
`r60a/protocol.py` if you obtain a datasheet that settles it the other way.

**`0x07 0x07` is unidentified.** A one-byte payload toggling `00`/`01` that did
not appear in the original capture but shows up in live sessions. It behaves
like a binary state flag and may well be the presence report. It is deliberately
*not* in `MESSAGE_MAP` — it surfaces as unidentified with its raw bytes rather
than being guessed at. Resolving this is the most useful open contribution.

**`0x80 0x01` (presence) has never been observed.** It is in the map from the
family convention, marked `verified=False`, and is presumably emitted only on
state change. Because of this the dashboard's occupancy tile falls back to
motion state and says on its face that the value is inferred rather than
reported.

**Long-format CSV over wide.** Fields arrive at different rates; a wide table
would be mostly empty cells.

**No web framework, no charting library.** The dashboard is one self-contained
HTML file served by `http.server`. It works offline, has no build step, no
`node_modules`, and no supply chain. Charts are hand-rolled SVG.

**Bad-checksum frames are kept, not dropped.** A silent drop hides a
deteriorating cable. They are returned flagged and counted, and the dashboard
turns its health indicator red when the count is non-zero.

---

## Privacy note

`data/sample_session.csv` and `tests/fixtures/capture_115200.bin` contain real
heart-rate and breathing-rate measurements recorded from a consenting adult
during development. They are included so the toolkit is usable and testable
without hardware. If you record your own sessions, remember that vital-signs
data may be sensitive: `data/*.csv` is gitignored by default for exactly this
reason.

---

## License

MIT — see [LICENSE](LICENSE).
