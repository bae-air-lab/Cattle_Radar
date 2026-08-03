"""Protocol tests built from hand-written byte strings plus a real capture."""

from __future__ import annotations

from pathlib import Path

import pytest

from r60a.protocol import (
    MESSAGE_MAP,
    Frame,
    FrameParser,
    Reading,
    UnknownFrame,
    checksum,
    decode,
    encode_frame,
)

FIXTURE = Path(__file__).parent / "fixtures" / "capture_115200.bin"

# Real frames lifted from the Phase 1 capture.
HEART_RATE_FRAME = bytes.fromhex("535985020001477b5443")  # 0x47 = 71 bpm
BREATHING_FRAME = bytes.fromhex("53598102000116465443")  # 0x16 = 22 breaths/min
DISTANCE_FRAME = bytes.fromhex("535980040002003b6d5443")  # 0x003b = 59 cm
POSITION_FRAME = bytes.fromhex("535980050006801f00320000085443")  # x=-31 y=50 z=0


def test_checksum_matches_hand_computed_value() -> None:
    # 0x53+0x59+0x80+0x02+0x00+0x01+0x01 = 0x130, masked to 0x30.
    assert checksum(bytes.fromhex("53598002000101")) == 0x30


def test_encode_frame_round_trips() -> None:
    raw = encode_frame(0x80, 0x04, bytes.fromhex("003b"))
    assert raw == DISTANCE_FRAME
    parser = FrameParser()
    (frame,) = parser.feed(raw)
    assert frame.control == 0x80
    assert frame.command == 0x04
    assert frame.payload == bytes.fromhex("003b")
    assert frame.valid_checksum


def test_clean_frame() -> None:
    parser = FrameParser()
    frames = parser.feed(BREATHING_FRAME)
    assert len(frames) == 1
    frame = frames[0]
    assert frame.key == (0x81, 0x02)
    assert frame.payload == b"\x16"
    assert frame.valid_checksum
    assert parser.stats.frames_ok == 1
    assert parser.stats.bytes_dropped == 0


def test_frame_split_across_two_reads() -> None:
    parser = FrameParser()
    first, second = BREATHING_FRAME[:4], BREATHING_FRAME[4:]

    assert parser.feed(first) == []
    assert parser.buffered == 4

    frames = parser.feed(second)
    assert len(frames) == 1
    assert frames[0].payload == b"\x16"
    assert parser.buffered == 0
    assert parser.stats.bytes_dropped == 0


@pytest.mark.parametrize("split", range(1, len(BREATHING_FRAME)))
def test_frame_split_at_every_possible_boundary(split: int) -> None:
    """One read() is never one frame; every split point must still work."""
    parser = FrameParser()
    frames = parser.feed(BREATHING_FRAME[:split]) + parser.feed(BREATHING_FRAME[split:])
    assert len(frames) == 1
    assert frames[0].key == (0x81, 0x02)


def test_frame_delivered_one_byte_at_a_time() -> None:
    parser = FrameParser()
    collected: list[Frame] = []
    for index in range(len(DISTANCE_FRAME)):
        collected.extend(parser.feed(DISTANCE_FRAME[index : index + 1]))
    assert len(collected) == 1
    assert collected[0].payload == bytes.fromhex("003b")


def test_leading_garbage_before_valid_frame() -> None:
    parser = FrameParser()
    garbage = bytes.fromhex("00ff53aa9912")
    frames = parser.feed(garbage + BREATHING_FRAME)
    assert len(frames) == 1
    assert frames[0].key == (0x81, 0x02)
    assert parser.stats.bytes_dropped == len(garbage)
    assert parser.stats.resyncs >= 1


def test_false_header_with_absurd_length_resyncs() -> None:
    """0x53 0x59 followed by a huge length must not swallow the real frame."""
    parser = FrameParser()
    decoy = bytes.fromhex("5359ffffffff")
    frames = parser.feed(decoy + BREATHING_FRAME)
    assert len(frames) == 1
    assert frames[0].key == (0x81, 0x02)


def test_overlapping_header_bytes_resync() -> None:
    """53 53 59 ... must land on the real header, so resync steps one byte."""
    parser = FrameParser()
    frames = parser.feed(b"\x53" + BREATHING_FRAME)
    assert len(frames) == 1
    assert frames[0].key == (0x81, 0x02)


def test_corrupted_checksum_is_kept_and_counted() -> None:
    corrupted = bytearray(BREATHING_FRAME)
    corrupted[-3] ^= 0xFF  # flip the checksum byte
    parser = FrameParser()
    frames = parser.feed(bytes(corrupted))

    assert len(frames) == 1, "bad-checksum frames must not be dropped"
    assert frames[0].valid_checksum is False
    assert frames[0].payload == b"\x16"
    assert parser.stats.frames_bad_checksum == 1
    assert parser.stats.frames_ok == 0


def test_corrupted_payload_is_detected() -> None:
    corrupted = bytearray(BREATHING_FRAME)
    corrupted[6] = 0x99  # payload byte, checksum left alone
    parser = FrameParser()
    (frame,) = parser.feed(bytes(corrupted))
    assert frame.valid_checksum is False


def test_two_back_to_back_frames_in_one_chunk() -> None:
    parser = FrameParser()
    frames = parser.feed(BREATHING_FRAME + HEART_RATE_FRAME)
    assert len(frames) == 2
    assert [f.key for f in frames] == [(0x81, 0x02), (0x85, 0x02)]
    assert all(f.valid_checksum for f in frames)
    assert parser.buffered == 0


def test_three_frames_with_garbage_between_them() -> None:
    parser = FrameParser()
    stream = BREATHING_FRAME + b"\xde\xad" + HEART_RATE_FRAME + b"\x53" + DISTANCE_FRAME
    frames = parser.feed(stream)
    assert [f.key for f in frames] == [(0x81, 0x02), (0x85, 0x02), (0x80, 0x04)]


def test_reset_clears_partial_frame() -> None:
    parser = FrameParser()
    parser.feed(BREATHING_FRAME[:5])
    assert parser.buffered == 5
    parser.reset()
    assert parser.buffered == 0
    assert parser.feed(BREATHING_FRAME) != []


def test_buffer_is_bounded_against_endless_garbage() -> None:
    parser = FrameParser(max_buffer=256)
    parser.feed(b"\x53\x59\x00\x01" * 4096)
    assert parser.buffered <= 256


# --------------------------------------------------------------------------
# decoding
# --------------------------------------------------------------------------


def _one(raw: bytes) -> Frame:
    parser = FrameParser()
    (frame,) = parser.feed(raw)
    return frame


def test_decode_breathing_rate() -> None:
    readings = decode(_one(BREATHING_FRAME))
    assert isinstance(readings, list)
    assert [(r.field_name, r.value) for r in readings] == [("breathing_rate_bpm", 22)]


def test_decode_heart_rate() -> None:
    readings = decode(_one(HEART_RATE_FRAME))
    assert isinstance(readings, list)
    assert [(r.field_name, r.value) for r in readings] == [("heart_rate_bpm", 0x47)]


def test_decode_distance_in_centimetres() -> None:
    readings = decode(_one(DISTANCE_FRAME))
    assert isinstance(readings, list)
    (reading,) = readings
    assert reading.field_name == "target_distance_cm"
    assert reading.value == pytest.approx(59.0)


def test_decode_position_uses_sign_magnitude() -> None:
    """0x801f is -31, not the -32737 two's complement would give."""
    readings = decode(_one(POSITION_FRAME))
    assert isinstance(readings, list)
    values = {r.field_name: r.value for r in readings}
    assert values == {"target_x_cm": -31, "target_y_cm": 50, "target_z_cm": 0}


def test_unknown_pair_is_surfaced_not_guessed() -> None:
    raw = encode_frame(0x99, 0x77, b"\x01\x02")
    result = decode(_one(raw))
    assert isinstance(result, UnknownFrame)
    assert result.key == (0x99, 0x77)
    assert result.payload_hex == "01 02"
    assert "53 59 99 77" in result.raw_hex


def test_short_payload_for_known_pair_is_unknown_not_truncated() -> None:
    """A 1-byte payload on the 6-byte position message must not be decoded."""
    raw = encode_frame(0x80, 0x05, b"\x01")
    assert isinstance(decode(_one(raw)), UnknownFrame)


def test_bad_checksum_flag_propagates_into_readings() -> None:
    corrupted = bytearray(BREATHING_FRAME)
    corrupted[-3] ^= 0xFF
    readings = decode(_one(bytes(corrupted)))
    assert isinstance(readings, list)
    assert all(r.valid_checksum is False for r in readings)


def test_enum_display_value() -> None:
    reading = Reading(
        timestamp=0.0,
        control=0x80,
        command=0x02,
        message="motion_state",
        field_name="motion_state",
        value=2,
        unit="",
        raw_hex="",
    )
    assert reading.display_value == "2 (active)"


# --------------------------------------------------------------------------
# regression against the real Phase 1 capture
# --------------------------------------------------------------------------


@pytest.fixture(scope="module")
def capture() -> bytes:
    if not FIXTURE.exists():
        pytest.skip(f"capture fixture missing: {FIXTURE}")
    return FIXTURE.read_bytes()


def test_capture_parses_completely(capture: bytes) -> None:
    parser = FrameParser()
    frames = parser.feed(capture)

    assert parser.stats.frames_ok == 86
    assert parser.stats.frames_bad_checksum == 0
    assert parser.stats.bytes_dropped == 0, "every byte should belong to a frame"
    assert parser.buffered <= 4, "only a trailing partial frame may remain"
    assert len(frames) == 86


def test_capture_parses_the_same_when_chunked(capture: bytes) -> None:
    """Chunk boundaries must not change the result -- the real read() case."""
    whole = FrameParser().feed(capture)

    for size in (1, 3, 7, 64, 512):
        parser = FrameParser()
        chunked: list[Frame] = []
        for offset in range(0, len(capture), size):
            chunked.extend(parser.feed(capture[offset : offset + size]))
        assert [f.raw for f in chunked] == [f.raw for f in whole], f"chunk size {size}"
        assert parser.stats.frames_bad_checksum == 0


def test_capture_contains_the_expected_message_types(capture: bytes) -> None:
    frames = FrameParser().feed(capture)
    seen = {f.key for f in frames}
    assert seen == {
        (0x01, 0x01),
        (0x80, 0x02),
        (0x80, 0x03),
        (0x80, 0x04),
        (0x80, 0x05),
        (0x81, 0x02),
        (0x85, 0x02),
    }
    assert seen <= set(MESSAGE_MAP), "capture contains a pair missing from MESSAGE_MAP"


def test_capture_decodes_to_plausible_vitals(capture: bytes) -> None:
    frames = FrameParser().feed(capture)
    values: dict[str, list[float]] = {}
    for frame in frames:
        result = decode(frame)
        assert not isinstance(result, UnknownFrame), f"undecoded {frame.key}"
        for reading in result:
            values.setdefault(reading.field_name, []).append(float(reading.value))

    assert all(4 <= v <= 60 for v in values["breathing_rate_bpm"])
    assert all(40 <= v <= 180 for v in values["heart_rate_bpm"])
    assert all(0 < v <= 600 for v in values["target_distance_cm"])
    assert all(-300 <= v <= 300 for v in values["target_x_cm"])
    assert all(0 <= v <= 100 for v in values["body_movement"])
