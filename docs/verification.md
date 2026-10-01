# Verification record — v0.1.0

Bench date: **2026-09-30**. Functional testing used one **Seeed Studio XIAO nRF54L15 Sense**, USB powered on Windows. Reference SDK: **NCS v3.3.0** (`ba167d9f3db4`), Zephyr **4.3.99** (`fd9204a02d52`), matching Nordic toolchain, board `xiao_nrf54l15/nrf54l15/cpuapp`.

## Software checks

- **18 native CTest suites** passed with Clang, Ninja and the MSVC runtime; warnings are errors.
- **24 Python tests** passed with Python 3.13, including **1,028 independent C/Python packet vectors**; interoperability ran without skipping.
- **18 firmware configurations** built from standalone release sources. OpenFloat also built through automatic Zephyr-module discovery using `ZEPHYR_EXTRA_MODULES`, without duplicate targets.
- GitHub CI covers GCC/Python 3.10 and Clang/Python 3.13; the workflow records its actual status.

Tests use production code with hardware/OS doubles. They exercise failure/recovery contracts without emulating silicon timing, radio coexistence or power-loss storage behavior. See [testing](testing.md) for coverage.

## Physical functional checks

Only **D10→D9** and **D6→D7** jumpers were installed. No external UART adapter, reference meter, oscilloscope or current meter was used. Every final standalone image received an explicit RRAM buffer commit and a complete SWD readback comparison against its HEX. Counts exclude preceding firmware output before the current boot.

| Check | Final observation | Limit |
| --- | --- | --- |
| Direct IMU | 33 motion snapshots in 5 s; progressing sequence/timestamps and plausible gravity | Nominal 104 Hz; no reference rotation/range fixture |
| FIFO/filter | 477 snapshots in 35 s, through sequence 14,871; epoch stayed 0 with no FIFO fault | Nominal 416 Hz; fetch times are not acquisition times; no long-duration rate calibration |
| INT1 FIFO | 11 summaries in 12 s; last complete summary: 4,688 samples, 4,530 IRQ callbacks, maximum batch 21 | Callback counts are not acquisition timestamps; cleanup faults not physically injected |
| Suspend/resume | Three completed cycles in 10 s, each with 104 samples and 2 s suspension; suspended reads rejected | Shared rail remained powered; current unmeasured |
| Calibration | 208-sample window completed with zero motion/gap resets; 34 corrected snapshots in 8 s | Functional estimate; absolute bias accuracy and field behavior uncharacterized |
| Settings/ZMS | Known signed bias saved, restored after software reboot, compared, saved again, cleared; absent key confirmed | Controlled bench used production APIs; no power-removal/interrupted-write/endurance testing |
| Packet/UART | All 257 payload lengths, 16 valid recovery packets and 16 malformed rejection probes; 35,552 received wire bytes, no overflow or line errors | Local jumper; independent external-adapter echo path untested |
| User button | Physical B press/release produced both debounced edges | 20 ms debounce, 5 ms polling; SDK calls this USR/sw0 |
| Temperature | Six progressing readings in each 6 s run, for both default and MPSL MCU backends plus IMU | No reference calibration or active radio traffic in the MPSL run |
| Microphone | 274 summaries in 30 s; two 500-block sessions stopped with buffers returned and restarted; third session active | Printed blocks nonzero with no PCM rail samples; sound response/rate/SPL uncalibrated |
| Watchdog | Both slots fed eight times, slot 1 withheld, reset cause `0x10` observed; demonstration did not rearm | Requested 2 s timeout; exact elapsed time and stop/restart unmeasured |
| SPI | 120 matching transfers: five rounds of modes 0–3, lengths 1, 3, 16, 255, 256, 512 | Requested 2 MHz; CS/clock waveform and external-device timing unmeasured |
| UART queue | Five overflow/resync rounds; 1,600 bytes received, 960 intentionally discarded, five overflows | 115200 8N1; no injected electrical faults or extended throughput test |
| ADC | 30 successful raw/millivolt readings in 3 s on unconnected D0 | Acquisition/calibration path only; voltage accuracy unmeasured |
| Battery divider | Four successful switched reads in 4 s, reporting 4,006 mV | USB powered; no meter comparison or inference about battery presence |
| PWM | 17 successful updates in 4 s across requested 0–1000 permille | API calls only; no external LED/scope on D0 |
| OpenFloat serial | 2,821 valid frames in 55 s at approximately 52 Hz; zero malformed/nonunit frames, gaps or discontinuities | All four final device error/drop counters zero |
| OpenFloat BLE | 839 valid 20-byte frames; initial/restarted/reconnected segments approximately 52 Hz, zero gaps; stop yielded zero frames after settling | Zero calibration confirmed in serial, invalid command rejected with ATT 0x13, reconnect without reset restored streaming |
| Sensor startup | Two starts after sensor-supply control was forced low for 100 ms initialized and streamed successfully | GPIO output/direction readback confirmed low; not a whole-board or battery-only cold-boot test |

The persistence/packet/button test was a local diagnostic harness. It is excluded because it automatically writes a bench calibration key. The public persistent calibration example requires an explicit button request before writing.

## Findings corrected before release

1. **FIFO port access:** a twelve-byte address-increment burst consumed one FIFO word, then read adjacent registers. The driver now performs six two-byte data-port reads. Regression tests model silicon's address behavior and fail with the old implementation.
2. **Empty FIFO pattern:** silicon retained pattern 5 after the final word even when empty. Empty status now returns no data without validating stale pattern bits; full/overrun checks still take precedence.
3. **FIFO backlog:** an unconditional 10 ms sleep after a full 32-pair batch eventually overran. A full batch now immediately starts another drain; an incomplete batch sleeps.
4. **SPI/UART settings:** SPIM00 rejected 1 MHz, so the example requests 2 MHz. UART21's P2 pins produced framing/break errors until Nordic system events and Constant Latency were enabled. The application owns this power policy.
5. **Programming integrity:** the installed Seeed helper left its last buffered writes uncommitted. Eight incorrect trailing bytes caused a BLE startup bus fault. Commit, READY polling, read-only configuration and complete verification fixed the image. The [Tcl helper](../tools/nrf54l15_rram.tcl) and [flash instructions](hardware-testing.md) include that sequence; native OpenOCD verification also passed.
6. **Early sensor probing:** one first-boot upstream IMU initialization failed and remained unavailable. OpenFloat now defers initialization until a 100 ms supply-settling wait, checks its result, then configures the wrapper. Both subsequent sensor-rail startup trials passed.

## Remaining measurements

Absolute voltage/temperature accuracy, divider shutdown voltage, PWM/CS/clock waveforms, maximum SPI rate, current consumption, battery-only/RTT boot, microphone sound/rate/SPL calibration, interrupted writes, storage endurance, MCU temperature during active radio traffic, watchdog stop/restart, prolonged reception/capture, and field validation of impact/bias thresholds require fixtures or longer experiments. Optical user-LED behavior was not independently confirmed. Successful API calls do not establish waveform or measurement accuracy.

The original **1,462,272-byte** application/persistent-data image was restored, native OpenOCD verification passed, and its complete readback matched the original SHA-256. After the original application's required reset, Head IMU booted, calibrated and emitted **326 numeric orientation records in an 8 s capture**. Raw captures, device identifiers and the original backup remain local and are excluded from this repository.

## Release-source firmware builds

Sizes are SDK linker used bytes, not runtime headroom or wire payload sizes. All builds used NCS v3.3.0; only expected Partition Manager deprecation warnings remained.

| Configuration | Flash bytes | RAM bytes |
| --- | ---: | ---: |
| `openfloat_telemetry` (Zephyr module) | 144,860 | 44,292 |
| `lsm6ds3tr_poll` | 39,128 | 8,096 |
| `adc_poll` | 38,392 | 8,056 |
| `imu_calibration` | 45,168 | 8,096 |
| `imu_fifo` | 43,968 | 8,096 |
| `pwm_output` | 35,220 | 7,824 |
| `watchdog_recovery` | 34,932 | 7,840 |
| `microphone_levels` | 39,380 | 11,184 |
| `battery_poll` | 39,048 | 8,096 |
| `spi_loopback` | 37,720 | 13,208 |
| `uart_loopback` | 37,272 | 8,208 |
| `uart_packet_echo` | 37,752 | 9,384 |
| `imu_fifo_irq` | 40,732 | 8,176 |
| `imu_power_cycle` | 39,180 | 8,096 |
| `temperature_poll` | 39,208 | 8,144 |
| `imu_calibration` + `persistent.conf` | 56,004 | 8,288 |
| `battery_poll` + `standalone.conf` | 38,124 | 9,256 |
| `temperature_poll` + `mpsl.conf` | 48,044 | 11,176 |
