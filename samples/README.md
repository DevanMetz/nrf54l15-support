# Firmware examples

All examples target the NCS v3.3.0 `xiao_nrf54l15/nrf54l15/cpuapp` board. Build one at a time into a separate directory. Every flash replaces the application's code. Serial console: USB debug UART, 115200 baud, DTR/RTS asserted when required.

| Example | Purpose | Fixture or interaction |
| --- | --- | --- |
| [lsm6ds3tr_poll](lsm6ds3tr_poll/) | Direct IMU polling at 104 Hz with SI-to-serial scaling | Sense onboard IMU |
| [imu_fifo](imu_fifo/) | Buffered 416 Hz paired motion, filter and continuity epochs | Sense onboard IMU |
| [imu_fifo_irq](imu_fifo_irq/) | FIFO watermark, GPIO callback and bounded wake/drain | Sense onboard IMU INT1 |
| [imu_power_cycle](imu_power_cycle/) | Capture, register suspend for 2 s, resume | Sense onboard IMU; separate meter for current |
| [imu_calibration](imu_calibration/) | Stationary bias estimation and corrected orientation | Keep the board still during calibration |
| [adc_poll](adc_poll/) | Raw/millivolt ADC readings on D0/A0/P1.04 | Known safe voltage and reference meter for accuracy |
| [battery_poll](battery_poll/) | Switched onboard battery-divider measurement | Battery and meter for absolute accuracy |
| [pwm_output](pwm_output/) | PWM on D0/P1.04 | External LED/resistor or scope |
| [watchdog_recovery](watchdog_recovery/) | Feed two progress slots, stall one, verify one watchdog reset | Intentionally resets, then leaves watchdog unarmed |
| [microphone_levels](microphone_levels/) | PCM levels and repeated capture/stop/restart | Sense microphone; sound source |
| [spi_loopback](spi_loopback/) | Four SPI modes and lengths through 512 bytes | Jumper D10/MOSI to D9/MISO; D8=SCK, D0=CS |
| [uart_loopback](uart_loopback/) | UART byte order, intentional overflow, resync/recovery | Jumper D6/TX/P2.08 to D7/RX/P2.07 |
| [uart_packet_echo](uart_packet_echo/) | COBS/CRC packet echo after bounded decoding | External 3.3 V UART adapter on D6/D7 and GND |
| [temperature_poll](temperature_poll/) | MCU/IMU temperatures with independent fetch timestamps | Sense IMU; optional MPSL configuration |
| [openfloat_telemetry](openfloat_telemetry/) | IMU/filter, impact candidates, LED/button, queued serial/BLE | Sense board; BLE client for controls/reconnect |

Optional configurations:

```sh
# Hold the user button during reset for the first explicit calibration/save.
west build -b xiao_nrf54l15/nrf54l15/cpuapp -d build/calibration-persistent \
  samples/imu_calibration -- -DEXTRA_CONF_FILE=persistent.conf

# UART disabled; nonblocking RTT output for battery-only bring-up.
west build -b xiao_nrf54l15/nrf54l15/cpuapp -d build/battery-standalone \
  samples/battery_poll -- -DEXTRA_CONF_FILE=standalone.conf

# MPSL initialized, with its alternate TEMP backend; no radio link started.
west build -b xiao_nrf54l15/nrf54l15/cpuapp -d build/temperature-mpsl \
  samples/temperature_poll -- -DEXTRA_CONF_FILE=mpsl.conf
```

The two loopback jumpers can remain installed during onboard sensor tests. Remove them when connecting external SPI/UART equipment. Do not drive an external voltage onto a pin configured as an output. Use the [hardware guide](../docs/hardware-testing.md) for expected logs and validation limits.
