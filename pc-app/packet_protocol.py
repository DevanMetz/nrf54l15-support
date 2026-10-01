"""Bounded COBS + CRC-32 packets used by samples/uart_packet_echo.

Wire: zero, COBS(payload + little-endian zlib CRC-32), zero.
Payloads contain 0..256 bytes. This is an envelope, not authentication or a
delivery protocol. Application types, versions and sequence numbers are opaque.
"""
from __future__ import annotations

import zlib

MAX_PAYLOAD = 256
MAX_ENCODED = 262
MAX_WIRE = 264


class BadPacket(ValueError):
    """Malformed COBS or a CRC mismatch."""


class PacketTooLarge(ValueError):
    """Encoded or decoded packet exceeds its bound."""


def encode(payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise PacketTooLarge("payload exceeds 256 bytes")
    raw = bytes(payload) + zlib.crc32(payload).to_bytes(4, "little")
    result = bytearray(b"\0")
    # Splitting into zero-separated runs provides a reference implementation
    # independent of the C encoder's one-pass code-byte backpatching.
    for run in raw.split(b"\0"):
        while len(run) >= 254:
            result.append(255)
            result.extend(run[:254])
            run = run[254:]
        result.append(len(run) + 1)
        result.extend(run)
    result.append(0)
    return bytes(result)


def decode_body(encoded: bytes) -> bytes:
    """Decode one COBS body without its leading/trailing zero delimiters."""
    if len(encoded) > MAX_ENCODED:
        raise PacketTooLarge("encoded packet exceeds 262 bytes")
    if not encoded or 0 in encoded:
        raise BadPacket("empty COBS body or embedded delimiter")
    raw = bytearray()
    position = 0
    while position < len(encoded):
        code = encoded[position]
        end = position + code
        if end > len(encoded):
            raise BadPacket("truncated COBS block")
        raw.extend(encoded[position + 1:end])
        position = end
        if code != 255 and position < len(encoded):
            raw.append(0)
        if len(raw) > MAX_PAYLOAD + 4:
            raise PacketTooLarge("decoded packet exceeds 260 bytes")
    if len(raw) < 4:
        raise BadPacket("packet has no complete CRC")
    payload = bytes(raw[:-4])
    if zlib.crc32(payload) != int.from_bytes(raw[-4:], "little"):
        raise BadPacket("CRC mismatch")
    return payload


class Decoder:
    """Feed arbitrary serial chunks; counters retain rejected-frame totals.

    reset() discards through the next zero, including after upstream byte loss.
    An oversized nonzero run counts once. Repeated delimiters count as idle.
    Stored partial input never exceeds MAX_ENCODED. No timeout is implicit.
    """

    def __init__(self) -> None:
        self.bad_frames = 0
        self.oversized_frames = 0
        self._encoded = bytearray()
        self._discard = True

    def reset(self) -> None:
        self._encoded.clear()
        self._discard = True

    @property
    def pending_bytes(self) -> int:
        """Encoded bytes in the current incomplete packet, never over 262."""
        return len(self._encoded)

    def feed(self, data: bytes) -> list[bytes]:
        packets: list[bytes] = []
        for byte in data:
            if byte == 0:
                if not self._discard and self._encoded:
                    try:
                        packets.append(decode_body(self._encoded))
                    except BadPacket:
                        self.bad_frames += 1
                    except PacketTooLarge:
                        self.oversized_frames += 1
                self._encoded.clear()
                self._discard = False
            elif not self._discard:
                if len(self._encoded) == MAX_ENCODED:
                    self._encoded.clear()
                    self._discard = True
                    self.oversized_frames += 1
                else:
                    self._encoded.append(byte)
        return packets
