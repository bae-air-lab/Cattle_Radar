from __future__ import annotations

from types import SimpleNamespace

from r60a.discovery import autodetect_port, list_candidate_ports
from r60a.visualize import _resolve_port


def test_linux_tty_ama_is_treated_as_candidate(monkeypatch) -> None:
    monkeypatch.setattr(
        "r60a.discovery.list_ports.comports",
        lambda: [
            SimpleNamespace(
                device="/dev/ttyAMA0",
                description="n/a",
                hwid="n/a",
            )
        ],
    )

    candidates = list_candidate_ports()

    assert candidates
    assert candidates[0].device == "/dev/ttyAMA0"


def test_autodetect_port_prefers_linux_tty_device_when_visible(monkeypatch) -> None:
    monkeypatch.setattr(
        "r60a.discovery.list_ports.comports",
        lambda: [
            SimpleNamespace(
                device="/dev/ttyAMA0",
                description="n/a",
                hwid="n/a",
            )
        ],
    )

    assert autodetect_port() == "/dev/ttyAMA0"


def test_visualize_resolve_port_returns_autodetected_device_name(monkeypatch) -> None:
    monkeypatch.setattr("r60a.visualize.autodetect_port", lambda: "/dev/ttyUSB0")

    assert _resolve_port("auto") == "/dev/ttyUSB0"
