# Changelog

## 0.1.0 — 2026-09-30

Initial experimental release:

- Portable orientation, telemetry, debounce, impact, fractional timing, calibration, bias-record, PCM16, CRC32 and packet libraries.
- Zephyr IMU, GPIO/button, ADC/battery, PWM, watchdog, PDM microphone, SPI, UART, temperature, Settings and OpenFloat BLE adapters.
- Direct LSM6DS3TR-C polling/FIFO, INT1 watermark notification and explicit suspend/resume.
- Python protocol codecs and serial validation/probe tools.
- Fifteen firmware examples, including the OpenFloat application integration.
- Native C and Python regression suites, CI, hardware testing instructions and validation records.
- Hardware-discovered FIFO fixes: read six individual words and ignore stale pattern bits when empty; drain another full batch without an unconditional sleep.
- XIAO loopback settings corrected to a representable 2 MHz SPI request and Constant Latency for UART21's P2 pin domain.
- OpenFloat defers IMU initialization until after sensor-supply settling; an OpenOCD Tcl helper commits and checks completion of final buffered RRAM writes.

The SDK baseline is NCS v3.3.0. API stability and hardware validation scope are described in the README and verification record.
