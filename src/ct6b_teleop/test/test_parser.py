import pytest


def parse_packet(packet: bytearray):
    if len(packet) < 18 or packet[0] != 0x55 or packet[1] != 0xFC:
        return None

    channels = []
    for j in range(7):
        high = packet[2 + 2 * j]
        low = packet[3 + 2 * j]
        channels.append((high << 8) | low)

    ch_sum = sum(packet[2:16])
    expected_high = ch_sum // 256
    expected_low = ch_sum % 256

    if expected_high == packet[16] and expected_low == packet[17]:
        return channels
    return None


def map_axis(raw_val: int, center: int = 1500, deadzone: int = 40,
             min_val: int = 1000, max_val: int = 2000, invert: bool = False) -> float:
    delta = raw_val - center
    if abs(delta) <= deadzone:
        return 0.0

    if delta > 0:
        span = max_val - (center + deadzone)
        norm = (delta - deadzone) / span if span > 0 else 1.0
        norm = min(1.0, max(0.0, norm))
    else:
        span = (center - deadzone) - min_val
        norm = (delta + deadzone) / span if span > 0 else -1.0
        norm = max(-1.0, min(0.0, norm))

    return -norm if invert else norm


def make_packet(channels):
    pkt = bytearray([0x55, 0xFC])
    for ch in channels:
        pkt.append((ch >> 8) & 0xFF)
        pkt.append(ch & 0xFF)
    ch_sum = sum(pkt[2:16])
    pkt.append(ch_sum // 256)
    pkt.append(ch_sum % 256)
    return pkt


def test_parse_valid_packet():
    test_channels = [1458, 1512, 1068, 1532, 1001, 1030, 142]
    pkt = make_packet(test_channels)
    parsed = parse_packet(pkt)
    assert parsed == test_channels


def test_parse_corrupted_checksum():
    test_channels = [1500, 1500, 1000, 1500, 1000, 1000, 0]
    pkt = make_packet(test_channels)
    pkt[16] ^= 0xFF  # Corrupt checksum
    assert parse_packet(pkt) is None


def test_deadzone():
    assert map_axis(1500) == 0.0
    assert map_axis(1530) == 0.0
    assert map_axis(1470) == 0.0
    assert map_axis(1540) == 0.0
    assert map_axis(1460) == 0.0


def test_limits():
    assert map_axis(2000) == pytest.approx(1.0)
    assert map_axis(1000) == pytest.approx(-1.0)


def test_inversion():
    assert map_axis(2000, invert=True) == pytest.approx(-1.0)
    assert map_axis(1000, invert=True) == pytest.approx(1.0)
