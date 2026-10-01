"""Dependency-free OpenFloat v1 wire decoder shared by host tools.

Text measurements preserve the firmware's integer units. BLE sequences are
16-bit and intentionally advance by eight in the default firmware.
"""

from dataclasses import dataclass
import re
import struct


LIVE_SERVICE_UUID = "8f3f3b10-0f5a-4f4c-9a2d-000000000001"
LIVE_CHARACTERISTIC_UUID = "8f3f3b10-0f5a-4f4c-9a2d-000000000002"
CONTROL_CHARACTERISTIC_UUID = "8f3f3b10-0f5a-4f4c-9a2d-000000000003"
BLE_FRAME = struct.Struct("<2sBBHH6h")
_INTEGER = re.compile(r"-?[0-9]+\Z")


def text_lines(chunks, max_length=512):
    """Reassemble newline-delimited serial bytes across read timeouts.

    Oversize lines emit one truncated diagnostic line (max_length + 1 bytes),
    then discard through the newline so the next frame can be decoded. An
    unfinished final line is omitted. Storage is bounded by max_length + 1.
    """
    if max_length < 1:
        raise ValueError("max_length must be positive")
    pending = bytearray()
    discarding = False
    for chunk in chunks:
        parts = chunk.split(b"\n")
        for index, part in enumerate(parts):
            if not discarding:
                pending.extend(part[:max_length + 1 - len(pending)])
                if len(pending) > max_length:
                    yield pending.decode("ascii", errors="replace")
                    pending.clear()
                    discarding = True
            if index < len(parts) - 1:
                if not discarding:
                    yield pending.decode("ascii", errors="replace")
                pending.clear()
                discarding = False


@dataclass(frozen=True)
class BleSample:
    sequence: int
    dt_us: int
    accel_mg: tuple[int, int, int]
    gyro_dps_q4: tuple[int, int, int]

    @property
    def gyro_dps(self) -> tuple[float, float, float]:
        return tuple(value / 16.0 for value in self.gyro_dps_q4)


@dataclass(frozen=True)
class LiveSample:
    sequence: int
    uptime_us: int
    dt_us: int
    accel_mg: tuple[int, int, int]
    gyro_mdps: tuple[int, int, int]
    angles_cdeg: tuple[int, int, int]
    quaternion_1e6: tuple[int, int, int, int]
    shot_count: int


@dataclass(frozen=True)
class ShotEvent:
    shot_id: int
    uptime_us: int
    accel_mg: tuple[int, int, int]
    shot_count: int


@dataclass(frozen=True)
class DeviceStats:
    """Cumulative firmware counters since its last reset, modulo 2**32."""

    queue_drops: int
    missed_ticks: int
    sample_errors: int
    ble_errors: int


def decode_ble(data: bytes | bytearray | memoryview) -> BleSample:
    """Decode exactly one 20-byte notification; reject other versions/types."""
    if len(data) != BLE_FRAME.size:
        raise ValueError(f"expected {BLE_FRAME.size} bytes, got {len(data)}")
    magic, version, kind, sequence, dt_us, *axes = BLE_FRAME.unpack(data)
    if (magic, version, kind) != (b"OF", 1, 1):
        raise ValueError("unsupported OpenFloat frame header")
    return BleSample(sequence, dt_us, tuple(axes[:3]), tuple(axes[3:]))


def _integer(token: str, low: int, high: int) -> int:
    # Bound length before int(), including on Python versions without an
    # integer-string conversion limit. Firmware emits at most 20 digits.
    if len(token) > 21 or not _INTEGER.fullmatch(token):
        raise ValueError(f"invalid integer: {token[:24]!r}")
    value = int(token)
    if not low <= value <= high:
        raise ValueError(f"integer outside [{low}, {high}]")
    return value


def decode_text(line: str) -> LiveSample | ShotEvent | DeviceStats | None:
    """Decode OFRAW/OFSHOT/# OFSTATS; ignore other logs and protocols.

    A recognized but malformed frame raises ValueError. This distinction lets
    serial tools count corruption without counting ordinary startup messages.
    """
    line = line.strip()
    kind = line.partition(",")[0]
    if kind not in ("OFRAW", "OFSHOT", "# OFSTATS"):
        return None
    if len(line) > 512:
        raise ValueError("OpenFloat text frame exceeds 512 characters")
    fields = line.split(",")
    expected = {"OFRAW": 19, "OFSHOT": 8, "# OFSTATS": 6}[kind]
    if len(fields) != expected:
        raise ValueError(f"{kind} requires {expected} fields, got {len(fields)}")
    if fields[1] != "1":
        raise ValueError("unsupported OpenFloat text version")
    u32 = lambda token: _integer(token, 0, 2**32 - 1)
    u64 = lambda token: _integer(token, 0, 2**64 - 1)
    i32 = lambda token: _integer(token, -(2**31), 2**31 - 1)
    if kind == "# OFSTATS":
        counters = {}
        for field in fields[2:]:
            key, separator, value = field.partition("=")
            if not separator or key not in DeviceStats.__dataclass_fields__ or key in counters:
                raise ValueError("invalid or duplicate OpenFloat counter name")
            counters[key] = u32(value)
        return DeviceStats(**counters)
    if kind == "OFSHOT":
        return ShotEvent(u32(fields[2]), u64(fields[3]),
                         tuple(i32(v) for v in fields[4:7]), u32(fields[7]))
    return LiveSample(u32(fields[2]), u64(fields[3]), u32(fields[4]),
                      tuple(i32(v) for v in fields[5:8]),
                      tuple(i32(v) for v in fields[8:11]),
                      tuple(i32(v) for v in fields[11:14]),
                      tuple(i32(v) for v in fields[14:18]), u32(fields[18]))


def missing_notifications(previous: int, current: int, *, bits: int = 16,
                          stride: int = 8) -> int | None:
    """Count missing decimated frames across sequence wrap.

    Return None for a duplicate, backward/reset sequence, or a stride mismatch.
    The caller should resynchronize after None; gaps spanning half the counter
    range cannot be distinguished from a reset using sequence numbers alone.
    """
    if bits not in (16, 32) or not 0 < stride < 2 ** (bits - 1):
        raise ValueError("use 16/32 bits and a positive stride below half-range")
    modulus = 2**bits
    if not (0 <= previous < modulus and 0 <= current < modulus):
        raise ValueError("sequence outside counter range")
    delta = (current - previous) % modulus
    if delta == 0 or delta >= modulus // 2 or delta % stride:
        return None
    return delta // stride - 1
