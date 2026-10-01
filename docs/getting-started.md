# Getting started

## Supported environment

The reference firmware environment is nRF Connect SDK **v3.3.0**, with its built-in **`xiao_nrf54l15/nrf54l15/cpuapp`** board. Sense-specific examples use the onboard LSM6DS3TR-C IMU and PDM microphone. Portable code has no SDK dependency. Other boards need their own devicetree wiring and have not been validated.

Install NCS and its matching toolchain using Nordic's tooling. Use the SDK shell so `west`, Zephyr and the cross-compiler are configured. Do not pass the old external Seeed board package as `BOARD_ROOT` in NCS v3.3.0.

## Checkout integration

In your application's `CMakeLists.txt`, after Zephyr discovery and `project(...)`:

```cmake
add_subdirectory(/absolute/path/to/nrf54l15-support/lib/nrf54l15
                 ${CMAKE_CURRENT_BINARY_DIR}/nrf54l15)
target_link_libraries(app PRIVATE nrf54l15_drivers)
```

Use `nrf54l15_core` for portable functionality and `nrf54l15_openfloat_ble` for the optional service. Include public headers as `<nrf54l15/packet.h>`, `<nrf54l15/imu.h>`, and so on. The [component table](../lib/nrf54l15/README.md#components) lists required Kconfig subsystems.

Do not configure a peripheral through two independent owners. In particular, direct IMU examples disable the upstream Zephyr IMU node while retaining the I2C bus and shared supply. State, callback objects and DMA storage must live for the full lifecycle described in their headers. Application code serializes access.

## West module integration

Add the library to your west manifest's `projects` list:

```yaml
- name: nrf54l15-support
  url: https://github.com/DevanMetz/nrf54l15-support
  revision: v0.1.0
  path: modules/lib/nrf54l15-support
```

After `west update`, Zephyr discovers `zephyr/module.yml`. Link the existing targets from your application; omit the explicit `add_subdirectory` call. The provided examples guard against duplicate target creation and work in either integration mode.

## Build examples

From a configured SDK shell:

```sh
west build -b xiao_nrf54l15/nrf54l15/cpuapp -d build/imu-fifo samples/imu_fifo
west build -b xiao_nrf54l15/nrf54l15/cpuapp -d build/temperature-poll samples/temperature_poll
west build -b xiao_nrf54l15/nrf54l15/cpuapp -d build/temperature-mpsl \
  samples/temperature_poll -- -DEXTRA_CONF_FILE=mpsl.conf
```

If this checkout is outside the west workspace and `west` does not expose the `build` command, run from the NCS workspace and replace the sample/build arguments with absolute paths. Use separate build directories for different configurations. Use a pristine build when changing board/overlay settings.

See the [example index](../samples/README.md) for pin connections and optional configurations. Console output uses the board's USB debug UART at 115200 baud; a serial client may need DTR/RTS asserted and a target reset after opening the port.

## Host tools

Python 3.10+ is required. Pure codecs and offline tests use the standard library. Live serial tools additionally require:

```sh
python -m pip install -r pc-app/requirements.txt
python pc-app/verify_openfloat.py --input capture.log
python pc-app/verify_openfloat.py --port COM11 --seconds 10 --capture capture.log
python pc-app/uart_packet_probe.py --help
```

`COM11` is only an example: enumerate and identify your current XIAO port. The packet probe connects to an external 3.3 V UART adapter wired to the packet example's UART pins, not to the separate USB console.
