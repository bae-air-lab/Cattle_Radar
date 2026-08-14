"""Gate and session tests.  No hardware and no UI: the gate is fed synthetic
readings on a synthetic clock, so 'the radar held its last value for 20 s' is
something we can assert on in milliseconds."""

from __future__ import annotations

import json
import time

import pytest

from r60a.protocol import FrameParser, encode_frame
from r60a.recorder import (
    GateConfig,
    RecorderCore,
    VitalsGate,
    sanitise_id,
)

T0 = 1_700_000_000.0


def fake_clock(monkeypatch: pytest.MonkeyPatch) -> dict[str, float]:
    """Freeze ``time.time`` on a dict we advance by hand.

    Patching the module attribute covers r60a.protocol too, so the frames the
    parser stamps share the same clock as the session.
    """
    clock = {"t": T0}
    monkeypatch.setattr(time, "time", lambda: clock["t"])
    return clock


def frame_for(kind: str, value: int):
    """Build a real frame so the tests exercise the parser and decode() too."""
    payloads = {
        "motion": (0x80, 0x02, bytes((value,))),
        "distance": (0x80, 0x04, value.to_bytes(2, "big")),
        "heart": (0x85, 0x02, bytes((value,))),
        "breath": (0x81, 0x02, bytes((value,))),
    }
    control, command, payload = payloads[kind]
    (frame,) = FrameParser().feed(encode_frame(control, command, payload))
    return frame


def feed_present(gate: VitalsGate, t: float, heart: int = 72, breath: int = 16) -> None:
    """One instant of a well-behaved present subject."""
    gate.note("motion_state", 2, t)
    gate.note("target_distance_cm", 60.0, t)
    gate.note("heart_rate_bpm", heart, t)
    gate.note("breathing_rate_bpm", breath, t)


def test_valid_when_target_present_and_vitals_move() -> None:
    gate = VitalsGate()
    for i in range(30):
        feed_present(gate, T0 + i, heart=70 + i % 4, breath=15 + i % 3)
    verdict = gate.evaluate(T0 + 29)
    assert verdict.valid
    assert verdict.occupied
    assert verdict.reasons == ()


def test_rejects_when_no_occupancy_evidence() -> None:
    gate = VitalsGate()
    # Vitals still arriving, but nothing says a target is there.
    for i in range(30):
        gate.note("heart_rate_bpm", 70 + i % 4, T0 + i)
        gate.note("breathing_rate_bpm", 16 + i % 3, T0 + i)
    verdict = gate.evaluate(T0 + 29)
    assert not verdict.valid
    assert not verdict.occupied
    assert "no_target" in verdict.reasons


def test_rejects_frozen_heart_rate() -> None:
    """The free-running failure: presence evidence and fresh frames, but the
    heart rate has not moved a single bpm in 30 s."""
    gate = VitalsGate()
    for i in range(30):
        gate.note("motion_state", 1, T0 + i)
        gate.note("target_distance_cm", 60.0, T0 + i)
        gate.note("heart_rate_bpm", 78, T0 + i)
        gate.note("breathing_rate_bpm", 15 + i % 3, T0 + i)
    verdict = gate.evaluate(T0 + 29)
    assert not verdict.valid
    assert "heart_rate_frozen" in verdict.reasons
    assert "breathing_rate_frozen" not in verdict.reasons


def test_freeze_needs_a_full_window_of_history() -> None:
    gate = VitalsGate()
    for i in range(8):  # 8 s of identical values, window is 20 s
        feed_present(gate, T0 + i, heart=78)
    assert not gate.frozen("heart_rate_bpm", T0 + 7)


def test_rejects_stale_vitals_after_the_frames_stop() -> None:
    gate = VitalsGate()
    for i in range(30):
        feed_present(gate, T0 + i, heart=70 + i % 4)
    verdict = gate.evaluate(T0 + 60)  # 30 s of silence
    assert not verdict.valid
    assert "heart_rate_stale" in verdict.reasons
    assert "no_target" in verdict.reasons


def test_rejects_out_of_range_and_zero_values() -> None:
    gate = VitalsGate()
    for i in range(30):
        feed_present(gate, T0 + i, heart=200 + i % 3, breath=0)
    verdict = gate.evaluate(T0 + 29)
    assert "heart_rate_out_of_range" in verdict.reasons
    assert "breathing_rate_zero" in verdict.reasons


def test_target_beyond_range_is_rejected() -> None:
    gate = VitalsGate(GateConfig(max_distance_cm=150.0))
    for i in range(30):
        gate.note("motion_state", 2, T0 + i)
        gate.note("target_distance_cm", 400.0, T0 + i)
        gate.note("heart_rate_bpm", 70 + i % 4, T0 + i)
        gate.note("breathing_rate_bpm", 16 + i % 3, T0 + i)
    verdict = gate.evaluate(T0 + 29)
    assert "target_too_far" in verdict.reasons


def test_reported_presence_overrides_inferred_occupancy() -> None:
    gate = VitalsGate()
    for i in range(30):
        feed_present(gate, T0 + i, heart=70 + i % 4)
        gate.note("presence", 0, T0 + i)  # module says nobody is there
    verdict = gate.evaluate(T0 + 29)
    assert not verdict.occupied
    assert gate.presence_reported


def test_disconnected_link_invalidates() -> None:
    gate = VitalsGate()
    for i in range(30):
        feed_present(gate, T0 + i, heart=70 + i % 4)
    verdict = gate.evaluate(T0 + 29, connected=False)
    assert not verdict.valid
    assert "disconnected" in verdict.reasons


def test_gate_disabled_still_records_reasons() -> None:
    gate = VitalsGate(GateConfig(enabled=False))
    verdict = gate.evaluate(T0)
    assert verdict.valid
    assert "no_target" in verdict.reasons


def test_reset_clears_history_across_a_reconnect() -> None:
    gate = VitalsGate()
    for i in range(30):
        feed_present(gate, T0 + i, heart=78)
    assert gate.frozen("heart_rate_bpm", T0 + 29)
    gate.reset()
    assert not gate.frozen("heart_rate_bpm", T0 + 29)
    assert gate.evaluate(T0 + 29).reasons  # nothing known any more


@pytest.mark.parametrize(
    "raw, expected",
    [
        ("cow 17", "cow_17"),
        ("  heifer/42  ", "heifer_42"),
        ("../etc/passwd", "etc_passwd"),
        ("", ""),
        ("...", ""),
    ],
)
def test_sanitise_id(raw: str, expected: str) -> None:
    assert sanitise_id(raw) == expected


def test_warmup_discards_then_recording_keeps(monkeypatch: pytest.MonkeyPatch) -> None:
    clock = fake_clock(monkeypatch)

    core = RecorderCore(warmup_s=15.0, sample_hz=1.0)
    core.set_connection(True, "test")
    core.start("cow_17")

    def beat(seconds: int, heart: int = 70) -> None:
        for _ in range(seconds):
            clock["t"] += 1.0
            core.ingest(frame_for("motion", 2))
            core.ingest(frame_for("distance", 60))
            core.ingest(frame_for("heart", heart + int(clock["t"]) % 4))
            core.ingest(frame_for("breath", 16 + int(clock["t"]) % 3))
            core.tick()

    beat(14)
    assert core.phase == "warmup"
    assert core.samples == []  # warm-up data is read but not kept

    beat(10)
    assert core.phase == "recording"
    assert 9 <= len(core.samples) <= 11
    assert all(s.valid for s in core.samples)

    document = core.stop()
    assert document["id"] == "cow_17"
    assert document["session"]["warmup_s"] == 15.0
    assert document["quality"]["samples_valid"] == len(document["samples"])
    assert document["summary"]["heart_rate_bpm"]["n"] == len(document["samples"])
    json.dumps(document)  # the document must be serialisable as-is


def test_absent_subject_records_samples_but_flags_them(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    clock = fake_clock(monkeypatch)

    core = RecorderCore(warmup_s=0.0, sample_hz=1.0)
    core.set_connection(True, "test")
    core.start("empty_room")

    # The module keeps publishing a latched heart rate with nobody in front of it.
    for _ in range(25):
        clock["t"] += 1.0
        core.ingest(frame_for("heart", 78))
        core.ingest(frame_for("breath", 15))
        core.tick()

    document = core.stop()
    assert document["quality"]["samples_valid"] == 0
    assert document["quality"]["samples_total"] > 0
    assert document["summary"]["heart_rate_bpm"] is None
    reasons = document["quality"]["reject_reasons"]
    assert "no_target" in reasons
    assert "heart_rate_frozen" in reasons


def test_drop_invalid_omits_rejected_samples(monkeypatch: pytest.MonkeyPatch) -> None:
    clock = fake_clock(monkeypatch)

    core = RecorderCore(warmup_s=0.0, sample_hz=1.0, keep_invalid=False)
    core.set_connection(True, "test")
    core.start("empty_room")
    for _ in range(10):
        clock["t"] += 1.0
        core.ingest(frame_for("heart", 78))
        core.tick()

    document = core.stop()
    assert document["samples"] == []
    assert document["quality"]["samples_rejected"] == 10
    assert document["quality"]["samples_total"] == 10
