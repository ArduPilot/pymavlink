"""Test replay-state commits in generated Python receivers."""

import hashlib
import importlib.util
from pathlib import Path

import pytest

from pymavlink.generator import mavgen


KEY = bytes(range(32))
BASE_TIMESTAMP = 30000000
STREAM_KEY = (7, 1, 1)


@pytest.fixture(scope="module", params=["Python", "Python3"])
def mav(tmp_path_factory, request):
    xml = Path(__file__).parent / "snapshottests/resources/minimal.xml"
    output = tmp_path_factory.mktemp("signing-state") / "minimal.py"
    assert mavgen.mavgen(
        mavgen.Opts(output=str(output), language=request.param,
                    wire_protocol="2.0", validate=False),
        [str(xml)],
    )
    spec = importlib.util.spec_from_file_location(
        "signing_state_minimal", output)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def receiver(mav, permissive):
    rx = mav.MAVLink(None)
    rx.signing.secret_key = KEY
    rx.signing.timestamp = BASE_TIMESTAMP
    if permissive:
        rx.signing.allow_unsigned_callback = lambda _link, _msgid: True
    return rx


def heartbeat(mav, timestamp, invalid=False):
    tx = mav.MAVLink(None, srcSystem=1, srcComponent=1)
    tx.signing.secret_key = KEY
    tx.signing.sign_outgoing = True
    tx.signing.link_id = STREAM_KEY[0]
    tx.signing.timestamp = timestamp
    message = mav.MAVLink_heartbeat_message(2, 3, 0, 0, 4, 3)
    packet = bytearray(message.pack(tx))
    assert hashlib.sha256(KEY + packet[:-6]).digest()[:6] == packet[-6:]
    if invalid:
        packet[-1] ^= 1  # Alter only the signature; leave the CRC intact.
        assert hashlib.sha256(KEY + packet[:-6]).digest()[:6] != packet[-6:]
    return bytes(packet)


def state(rx):
    return dict(rx.signing.stream_timestamps), rx.signing.timestamp


@pytest.mark.parametrize("permissive", [False, True])
@pytest.mark.parametrize("existing_stream", [False, True])
def test_invalid_signature_preserves_state_and_valid_suffix(
        mav, permissive, existing_stream):
    rx = receiver(mav, permissive)
    if existing_stream:
        assert rx.parse_char(heartbeat(mav, BASE_TIMESTAMP)) is not None
    before = state(rx)
    invalid = heartbeat(mav, BASE_TIMESTAMP + 10, invalid=True)
    if permissive:
        assert rx.parse_char(invalid) is not None
    else:
        with pytest.raises(mav.MAVError, match="signature"):
            rx.parse_char(invalid)
    assert state(rx) == before

    # Authentication failure must not prevent the next legitimate message.
    assert rx.parse_char(heartbeat(mav, BASE_TIMESTAMP + 1)) is not None
    assert rx.signing.stream_timestamps[STREAM_KEY] == BASE_TIMESTAMP + 1
    assert rx.signing.timestamp == BASE_TIMESTAMP + 1


@pytest.mark.parametrize("permissive", [False, True])
def test_valid_timestamps_advance_and_replay_preserves_state(mav, permissive):
    rx = receiver(mav, permissive)
    for timestamp in [BASE_TIMESTAMP + 1, BASE_TIMESTAMP + 2]:
        assert rx.parse_char(heartbeat(mav, timestamp)) is not None
        assert rx.signing.stream_timestamps[STREAM_KEY] == timestamp
        assert rx.signing.timestamp == timestamp
    before = state(rx)
    replay = heartbeat(mav, BASE_TIMESTAMP + 2)
    if permissive:
        assert rx.parse_char(replay) is not None
    else:
        with pytest.raises(mav.MAVError, match="signature"):
            rx.parse_char(replay)
    assert state(rx) == before
