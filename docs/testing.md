# Testing

## Native C and Python

```sh
cmake -S . -B build/native
cmake --build build/native --config Debug
ctest --test-dir build/native -C Debug --output-on-failure
python -m unittest discover -s tests/python -v
```

There are 18 CTest suites and 24 Python tests. Native compilation treats warnings as errors. The hardware adapters compile against test doubles for Zephyr interfaces; these tests exercise production library code without pretending to emulate the chip or radio.

For the C/Python packet interoperability test, point `NRF54L15_PACKET_TEST_BIN` to the built executable:

```sh
# Linux/macOS, with the root CMake build above:
export NRF54L15_PACKET_TEST_BIN="$PWD/build/native/tests/host/packet_tests"
python -m unittest discover -s tests/python -v
```

```powershell
# Ninja/Clang on Windows:
$env:NRF54L15_PACKET_TEST_BIN = "$PWD/build/native/tests/host/packet_tests.exe"
python -m unittest discover -s tests/python -v
```

Multi-configuration generators can put executables in `tests/host/Debug/`. Set the override to that actual location. You can also build `tests/host` directly into `build/host-tests`, which the Python test discovers automatically. The interoperability test skips when no executable is available; CI sets the override explicitly so it must run.

The Windows development baseline uses Clang with the MSVC runtime and Ninja. Linux CI runs GCC/Python 3.10 and Clang/Python 3.13. SDK tools and native compilers are separate: the nRF cross-compiler does not run the host executables.

## Coverage

Checks include bounded streaming/COBS recovery, independent CRC fixtures, 1,028 C/Python packet vectors, all 65,536 IMU temperature encodings, each configuration/power-transition bus failure, FIFO pattern/overrun recovery, DMA ownership/cleanup, UART overflow and IRQ races, and calibration motion/gap rejection.

| CTest suite | Coverage |
| --- | --- |
| `core` | Stationary/rotating filter behavior, invalid inputs, packet bytes and saturation, control parsing, debounce/clock wrap, impact hysteresis, fractional deadlines |
| `drivers` | Sensor and GPIO configuration, error propagation and partial failures |
| `lsm6ds3tr` | All range/rate combinations; failure of each configuration transaction; FIFO signed axes, ordered backlog, partial pairs, multi-byte counts, sequence wrap, transitions, partially consumed reads, overrun/pattern faults and recovery; INT1 route verification; suspend/resume at all ranges/rates, 100 cycles, each partial transition failure, stale-output discard and blocked acquisition; all 65,536 temperature register values, freshness/error handling and motion/FIFO independence |
| `adc` | Offset calibration and retry, DMA alignment, voltage conversion and invalid configuration |
| `calibration` | Motion/noise rejection, window restart, maximum window size and simulated yaw drift before/after bias correction |
| `ble` | Startup/retry, notifications, control validation, coalesced zero requests and deferred advertising after connection recycling |
| `pwm` | Duty bounds, monotonicity/rounding over uint32 periods, polarity, fixed configuration and initialization/driver failures |
| `gyro_bias` | Independent Python/zlib byte fixture; every single-bit corruption; truncated/oversized records; version/profile checks; quantization/range limits; Settings failures and a partially replaced value |
| `watchdog` | One to eight channels, pause options, timeout bounds, startup/stop/restart, partial allocation failures, feed errors and a stalled-worker model |
| `dmic` | Slab/configuration bounds, channel negotiation, failed/partial starts, malformed reads, delayed DMA release, queued and late blocks, empty-pool idle race, cleanup errors and 100 restarts |
| `pcm16` | Silence, DC, analytical sine/square signals, signed rails, 32-bit accumulation overflow cases and invalid input preservation |
| `battery` | Enable/settle/ADC/release ordering, each failure stage, retained-reference retry, other-owner protection, calibration retry, signed scaling/rounding and overflow |
| `spi` | Mode/bit-order/CS validation, stable copied configuration, every length 1–512, aligned private DMA buffers, dummy bytes, overlapping caller buffers, partial-write errors and separate device configurations |
| `uart_rx` | Byte ordering and queue wrap, partial reads, loss latch/resync, line errors, IRQ/FIFO faults, interrupt-enable races, failed callback removal, restart, counter saturation and packet recovery after UART loss at each frame offset |
| `packet` | Known CRC/COBS bytes, every payload length, all byte values, overlapping encode buffers, 3,000 random payloads, corruption/deletion/reset recovery, bounded malformed streams and unchanged output on failure |
| `gpio_event` | Supported-pin/polarity validation, already-active wake, notification coalescing, callback races, timeout/read errors, partial startup, cleanup retry, two pins on a shared port, 100 restarts and counter saturation |
| `temperature` | Signed quarter-degrees, rounding ties, representation overflow, fetch/get error propagation and unchanged output |
| `temperature_mpsl` | Same conversion/error checks with simulated MPSL lock ownership, contention/retry and release on backend failure |

Simulations do not prove realized sample rates, voltage or temperature accuracy, physical interrupt behavior, reset/current measurements, radio coexistence, SDK atomic ordering or power-loss persistence. Use [hardware testing](hardware-testing.md) and record observations in [verification](verification.md).

## Firmware checks

Build affected examples with NCS v3.3.0 after each driver change. Native tests cannot catch changed devicetree macros, backend structure layouts or SDK-selected alternative drivers. The default and MPSL temperature builds are separate; persistent calibration reserves its own Settings partition. Keep configurations in distinct build directories and inspect generated partition layouts before flashing persistent data.
