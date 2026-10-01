# nRF54L15 Support

[![Native tests](https://github.com/DevanMetz/nrf54l15-support/actions/workflows/native-tests.yml/badge.svg)](https://github.com/DevanMetz/nrf54l15-support/actions/workflows/native-tests.yml)
[![License: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)

C libraries and Zephyr hardware adapters for **Nordic nRF54L15**, with examples for the **Seeed Studio XIAO nRF54L15 Sense**. Use the portable code on a host or embed the drivers in an nRF Connect SDK application.

**v0.1.0 is an experimental first release.** The reference SDK is **nRF Connect SDK v3.3.0**, board `xiao_nrf54l15/nrf54l15/cpuapp`. Hardware validation results and remaining fixture requirements are recorded in [verification](docs/verification.md). Public APIs may change before v1.0.

## What it includes

| Area | Features |
| --- | --- |
| Orientation | Madgwick IMU filter, quaternion normalization, Euler conversion, wrapped angles |
| Motion | Upstream Zephyr IMU adapter; direct LSM6DS3TR-C I2C polling, FIFO, INT1 notifications, suspend/resume |
| Calibration | Stationary gyro bias with motion/noise rejection; versioned, CRC-protected records; optional Settings/ZMS persistence |
| Scheduling and events | Fractional sample deadlines, missed-slot accounting, debounce, impact candidates, GPIO wake notification |
| Analog and outputs | SAADC raw/millivolt reads and calibration, switched battery divider, PWM duty/pulse control |
| Audio and temperature | PDM mono PCM16 capture/cleanup, PCM levels, MCU and IMU die-temperature reads |
| Transport | Fixed-configuration SPI; UART receive buffering/loss recovery; COBS/CRC packets; OpenFloat BLE service |
| Host tools | Python reference codecs, serial telemetry validation, UART packet echo probe |

The library code allocates no heap memory. Callers own state and buffers; the BLE service is a singleton. Zephyr and the SDK manage their own resources. Hardware calls run in thread context and must be serialized. See the [API guide](lib/nrf54l15/README.md) and public [headers](lib/nrf54l15/include/nrf54l15/) for exact contracts, supported bounds, and recovery rules.

## Try the portable code and tests

Requirements: CMake 3.20+, a C99/C11 compiler, and Python 3.10+. Ninja is optional.

```sh
git clone https://github.com/DevanMetz/nrf54l15-support.git
cd nrf54l15-support
cmake -S . -B build/native
cmake --build build/native --config Debug
ctest --test-dir build/native -C Debug --output-on-failure
python -m unittest discover -s tests/python -v
```

The native suites build the production C code with simulated hardware/OS interfaces. The Python packet suite also compares 1,028 independent vectors with the built C encoder/decoder. See [testing](docs/testing.md) for explicit executable selection and platform notes.

To use just the portable library in a CMake project:

```cmake
add_subdirectory(path/to/nrf54l15-support/lib/nrf54l15 nrf54l15)
target_link_libraries(your_app PRIVATE nrf54l15_core)
```

## Build a firmware example

In a configured **NCS v3.3.0 shell**, with this checkout as your working directory:

```sh
west build -b xiao_nrf54l15/nrf54l15/cpuapp \
  -d build/temperature-poll samples/temperature_poll
```

If `west build` is unavailable outside the SDK workspace, run it from the NCS workspace and pass absolute paths for the sample and build directory. NCS v3.3.0 includes this board; do not add an external Seeed `BOARD_ROOT`.

Flash through the board's configured debug runner or follow [hardware testing](docs/hardware-testing.md). Every firmware example replaces the board's application. SPI/UART loopbacks require jumper wires; the watchdog example intentionally resets the board.

## Integrate with Zephyr

After `find_package(Zephyr ...)` and `project(...)` in your application's CMake file:

```cmake
add_subdirectory(path/to/nrf54l15-support/lib/nrf54l15 nrf54l15)
target_link_libraries(app PRIVATE nrf54l15_drivers)
# When using the OpenFloat service:
# target_link_libraries(app PRIVATE nrf54l15_openfloat_ble)
```

Enable the matching Zephyr subsystems and provide your board's devicetree configuration. Drivers are compiled conditionally on `CONFIG_SENSOR`, `CONFIG_I2C`, `CONFIG_GPIO`, `CONFIG_ADC`, and the other subsystem options. A `zephyr/module.yml` also permits use as a west module; when automatically loaded, link the existing targets instead of adding the same directory twice. See [getting started](docs/getting-started.md).

## Examples and further reading

- [Example index](samples/README.md): 15 applications and optional configurations.
- [API and integration guide](lib/nrf54l15/README.md): units, ownership, wire formats, SDK limitations, recovery, and build matrix.
- [Testing](docs/testing.md): native/Python checks and what their simulations establish.
- [Hardware testing](docs/hardware-testing.md): fixtures, console setup, and expected results.
- [Verification record](docs/verification.md): actual hardware evidence and remaining checks.
- [Contributing](CONTRIBUTING.md) and [changelog](CHANGELOG.md).

## License

[Apache License 2.0](LICENSE). Nordic SDK dependencies retain their own licenses and are installed separately.
