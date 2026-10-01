"""Check uart_packet_echo through an external 3.3 V UART adapter at 115200.

Connect adapter RX to XIAO D6/TX, adapter TX to D7/RX and common GND. Select
the adapter's port, not the XIAO debug console. The test sends binary patterns
and malformed frames; use only a board running the packet echo sample.
"""
from __future__ import annotations

import argparse
import math
import time

from packet_protocol import Decoder, MAX_ENCODED, encode


def _positive_int(value: str) -> int:
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return result


def _timeout(value: str) -> float:
    result = float(value)
    if not math.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return result


def _send(port, data: bytes) -> None:
    count = port.write(data)
    if count != len(data):
        raise OSError(f"partial UART write: {count}/{len(data)} bytes")


def _reply(port, decoder: Decoder, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    errors_before = decoder.bad_frames + decoder.oversized_frames
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("no complete echo before the deadline")
        port.timeout = min(0.05, remaining)
        data = port.read(max(1, min(264, port.in_waiting)))
        replies = decoder.feed(data)
        if decoder.bad_frames + decoder.oversized_frames != errors_before:
            raise OSError("malformed packet received from the board")
        if len(replies) > 1:
            raise OSError("more than one echo received for a request")
        if replies:
            if decoder.pending_bytes:
                raise OSError("unexpected partial packet after an echo")
            return replies[0]


def verify_echo(port, rounds: int = 257, timeout: float = 1.0) -> tuple[int, int]:
    """Send one request at a time; stop on mismatch, I/O failure or timeout.

    Every 17th round first checks rejection of a corrupt, truncated or oversized
    frame (cycling through those cases). The following valid request verifies
    resynchronization. A 75 ms quiet read lets the short invalid input drain.
    The caller opens the port; this function neither resets nor flashes a board.
    """
    if rounds <= 0 or not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("rounds and timeout must be positive and finite")
    decoder = Decoder()
    port.write_timeout = timeout
    port.reset_input_buffer()
    probes = 0
    for sequence in range(rounds):
        if sequence % 17 == 0:
            kind = probes % 3
            if kind == 0:
                damaged = bytearray(encode(b"123456789"))
                damaged[2] ^= 1  # Change a payload byte, leaving the CRC intact.
                invalid = bytes(damaged)
            elif kind == 1:
                invalid = encode(b"discard truncated")[:-3]
            else:
                invalid = b"\0" + b"\x01" * (MAX_ENCODED + 10) + b"\0"
            _send(port, invalid)
            port.timeout = 0.075
            if port.read(1):
                raise OSError("board sent data in response to an invalid frame")
            probes += 1
        length = sequence % 257
        payload = bytes((index * 37 + sequence * 17) & 255 for index in range(length))
        _send(port, encode(payload))
        echoed = _reply(port, decoder, timeout)
        if echoed != payload:
            raise OSError(f"echo mismatch in round {sequence}, expected {length} payload bytes")
    # Catch trailing duplicate bytes instead of hiding them at the next launch.
    port.timeout = 0.075
    if port.read(1):
        raise OSError("unexpected trailing bytes after the last echo")
    return rounds, probes


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="external UART adapter port, e.g. COM12")
    parser.add_argument("--rounds", type=_positive_int, default=257)
    parser.add_argument("--timeout", type=_timeout, default=1.0, help="seconds per write/reply")
    args = parser.parse_args()
    try:
        import serial
    except ImportError:
        parser.exit(2, "pyserial is required for live probing; install it in this Python environment.\n")
    try:
        with serial.Serial(args.port, 115200, timeout=0.05, write_timeout=args.timeout,
                           rtscts=False, dsrdtr=False, xonxoff=False) as port:
            frames, probes = verify_echo(port, args.rounds, args.timeout)
    except (OSError, ValueError, serial.SerialException) as error:
        print(f"FAIL: {error}")
        return 1
    print(f"PASS: {frames} echoed frames, {probes} invalid-frame recovery probes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
