# nRF54L15 support library

Reusable C libraries and Zephyr hardware adapters for the Seeed XIAO nRF54L15 Sense. The [`openfloat_telemetry`](../../samples/openfloat_telemetry/) example uses these libraries with NCS v3.3.0 and its built-in `xiao_nrf54l15/nrf54l15/cpuapp` board. Do not add the external Seeed `BOARD_ROOT` to that SDK.

## Components

| Header | Purpose | Runtime requirements |
| --- | --- | --- |
| `nrf54l15/orientation.h` | Madgwick IMU filter, quaternion normalization, Euler conversion, wrapped angle differences | Standard C, math library |
| `nrf54l15/telemetry.h` | OpenFloat v1 encoding/decoding, exact control command parsing, saturating integer scaling | Standard C, math library |
| `nrf54l15/debounce.h` | Debounced press/release edges, including clock wrap | Standard C |
| `nrf54l15/impact.h` | Acceleration threshold events with hysteresis and refractory time | Standard C, math library |
| `nrf54l15/periodic.h` | Fractional tick deadlines, drift-free scheduling, missed-slot accounting | Standard C |
| `nrf54l15/gyro_calibration.h` | Explicit stationary gyro bias estimate with acceleration, rate and variance limits | Standard C, math library |
| `nrf54l15/gyro_bias.h` | Versioned gyro bias record, CRC, profile matching, optional persistent load/save/clear | Codec: standard C; storage: `CONFIG_SETTINGS` |
| `nrf54l15/pcm16.h` | PCM16 peak, mean, RMS and rail-sample count with 64-bit accumulation | Standard C, math library |
| `nrf54l15/crc32.h` | Table-free CRC-32/ISO-HDLC shared by calibration storage and packets | Standard C |
| `nrf54l15/packet.h` | COBS/CRC packet encoding, bounded streaming decode and recovery after byte loss | Standard C, payloads up to 256 bytes |
| `nrf54l15/imu.h` | Explicit IMU range/rate configuration and SI-unit samples through Zephyr's sensor API | `CONFIG_SENSOR`, upstream LSM6DSL driver |
| `nrf54l15/temperature.h` | Signed millidegree samples and SoC die-temperature reads, with MPSL lock protection when enabled | `CONFIG_SENSOR`, enabled `temp` node; blocking SDK conversion |
| `nrf54l15/button.h` | Devicetree GPIO input with debounce and error propagation | `CONFIG_GPIO` |
| `nrf54l15/gpio_event.h` | Activation notification, already-active level checks, bounded waits and callback cleanup | `CONFIG_GPIO`, nrfx GPIO interrupts, exclusive pin, single application CPU |
| `nrf54l15/adc.h` | One-channel SAADC acquisition, offset calibration, raw and millivolt readings | `CONFIG_ADC` |
| `nrf54l15/battery.h` | Switched-divider measurement, power-reference cleanup and checked voltage scaling | `CONFIG_ADC`, `CONFIG_REGULATOR`, dedicated divider supply |
| `nrf54l15/spi.h` | Fixed-configuration SPI transactions with aligned buffers and preserved RX on errors | `CONFIG_SPI`, persistent instance, up to 512 bytes per transaction |
| `nrf54l15/uart_rx.h` | Interrupt-driven UART receive queue, explicit loss acknowledgment and coherent diagnostics | `CONFIG_UART_INTERRUPT_DRIVEN`, `CONFIG_RING_BUFFER`, exclusive UART, single application CPU |
| `nrf54l15/pwm.h` | Fixed-period PWM, duty in permille or pulse width in nanoseconds, devicetree polarity | `CONFIG_PWM`, output pin on P1 |
| `nrf54l15/watchdog.h` | Up to eight progress channels, shared watchdog timeout, guarded startup and explicit stop | `CONFIG_WATCHDOG`, exclusive peripheral ownership |
| `nrf54l15/dmic.h` | Mono PCM16 capture, copied blocks, bounded stop/drain and restart | `CONFIG_AUDIO`, `CONFIG_AUDIO_DMIC`, dedicated memory slab |
| `nrf54l15/lsm6ds3tr.h` | Direct LSM6DS3TR-C I2C motion/temperature, FIFO, INT1 watermark, suspend/resume and fault recovery | `CONFIG_I2C`, exclusive sensor ownership |
| `nrf54l15/openfloat_ble.h` | Existing OpenFloat GATT service, advertising lifecycle, notifications and zero/start/stop control | `CONFIG_BT_PERIPHERAL`, `CONFIG_BT_MAX_CONN=1` |

No heap allocation is used by the library code. RAM state belongs to the caller except for the singleton BLE service; gyro bias persistence uses one reserved Settings key. Hardware operations must run in thread context. Serialize all access to each driver/filter instance. The library depends on Zephyr's established nRF54L15 peripheral drivers instead of duplicating the SoC HAL.

## Integrate with a Zephyr application

After `find_package(Zephyr ...)` and `project(...)`:

```cmake
add_subdirectory(path/to/lib/nrf54l15 ${CMAKE_CURRENT_BINARY_DIR}/nrf54l15)
target_link_libraries(app PRIVATE nrf54l15_drivers m)
# For the OpenFloat BLE service, also link nrf54l15_openfloat_ble.
```

`nrf54l15_core` is usable independently, including outside Zephyr. `nrf54l15_drivers` compiles only adapters whose relevant Zephyr subsystem is enabled. `nrf54l15_openfloat_ble` is available when `CONFIG_BT_PERIPHERAL` is enabled. Generated-header dependencies are explicit, so a clean parallel build is supported.

The upstream IMU adapter expects runtime configuration:

```ini
CONFIG_SENSOR=y
CONFIG_LSM6DSL=y
CONFIG_LSM6DSL_ACCEL_FS=0
CONFIG_LSM6DSL_GYRO_FS=0
CONFIG_LSM6DSL_ACCEL_ODR=0
CONFIG_LSM6DSL_GYRO_ODR=0
```

```c
static struct nrf54l15_imu imu;
const struct nrf54l15_imu_config config = {
    .sample_hz = 416,
    .accel_range_g = 16,
    .gyro_range_dps = 2000,
};
int err = nrf54l15_imu_init(&imu, DEVICE_DT_GET(DT_ALIAS(imu0)), &config);
/* Handle err, then allow sensor settling before reading. */
```

The 16 g range permits the prototype's 12 g event threshold. The old default 2 g range could not measure that threshold. Scaling uses m/s² and radians/s inside the library. The application applies its mounting rotation, so drivers always report the physical sensor axes.

## Sensor transport choices

The active firmware retains the upstream Zephyr sensor driver. Its adapter preserves output on errors reported by `sensor_sample_fetch()` or `sensor_channel_get()`. **The installed NCS v3.3.0 LSM6DSL driver's `lsm6dsl_sample_fetch()` discards return values from its internal I2C reads.** Some bus failures can therefore appear as successful cached samples; the wrapper cannot detect errors the SDK hides.

The direct `lsm6ds3tr` driver addresses that gap. It checks chip ID, bounds reset polling, sets ranges and rates, enables block-data-update and address increment, verifies control-register readback, checks both data-ready bits, and reads gyro/accel in one burst. It returns `-EAGAIN` for no fresh data and propagates bus failures without publishing a partial sample. It supports 26–833 Hz at the six rates listed in its header. Optional FIFO acquisition, INT1 watermark routing and explicit suspend/resume are described below; SPI and automatic Zephyr power-management hooks are not implemented.

Use the standalone [`lsm6ds3tr_poll`](../../samples/lsm6ds3tr_poll/) sample to evaluate it. That sample disables the upstream sensor node while keeping the I2C controller and sensor supply enabled. Never let both drivers configure the same sensor. The direct backend has passed register-model tests and physical Sense-board polling, FIFO, interrupt and suspend/resume checks. The OpenFloat example continues to use the upstream backend; see the verification record for measured durations and limits.

Register definitions and scale factors follow [ST's LSM6DS3TR-C datasheet](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf), table 3 and sections 9.12–9.15, 9.27–9.40. The application continues to use the board's power configuration and console route described by [Zephyr's XIAO board documentation](https://docs.zephyrproject.org/latest/boards/seeed/xiao_nrf54l15/doc/index.html).

### Buffered IMU acquisition

After direct-driver initialization, `nrf54l15_lsm6ds3tr_fifo_start()` clears the backlog and enables continuous, equally paced gyro/accel data without decimation. `fifo_read()` retrieves one complete pair per call, checking status and the six-word axis pattern before and after six separate two-byte reads of FIFO_DATA_OUT_L/H. Address increment does not wrap back to that FIFO port after the high byte. Empty-FIFO pattern bits are ignored because silicon can retain a stale pattern when no words remain. `fifo_stop()` clears the FIFO and restores output-register polling. The first pair after either transition is discarded. The register sequence follows [ST AN5130, sections 8.3–8.5](https://www.st.com/resource/en/application_note/an5130-lsm6ds3trc-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf).

An incomplete pair returns `-EAGAIN`. Full/overrun status returns `-EOVERFLOW`; an unexpected pattern returns `-EILSEQ`. Bus and integrity faults leave output unchanged and latch the stream into `-EPIPE` until an explicit start or stop succeeds. A failed word read may already have consumed words, so an ordinary read retry cannot safely establish continuity. Starting again discards pending data and resets the host sequence to zero.

FIFO timestamps are host fetch times. Use the configured nominal sample period when processing a backlog; intervals between fetches do not represent acquisition intervals. Sequence numbers count published pairs within the current start/restart epoch. They cannot establish the exact number of physical samples lost. The release bench exercised continuous draining and pattern coherence. Oscillator accuracy and long-duration throughput still require characterization.

The [`imu_fifo`](../../samples/imu_fifo/) example processes each buffered pair through Madgwick at a nominal 416 Hz, drains at most 32 pairs per pass, and sleeps 10 ms only when fewer than 32 pairs were available. A full batch is followed immediately by another drain so the host does not fall behind. Its built-in board configuration uses a 400 kHz IMU bus. It prints occasional `IMUFIFO` snapshots and cumulative `FIFOSTATS` diagnostics. A FIFO fault starts a new epoch and resets orientation because the missing motion history is unknown. A one-second interval without complete samples ends the example with a timeout diagnostic. This example has no gyro bias correction; use the calibration helper when integrating the two workflows.

### Interrupt-driven FIFO acquisition

`nrf54l15_lsm6ds3tr_fifo_int1(sensor, true)` routes the FIFO watermark to INT1 using bit 3 of `INT1_CTRL`, with the active-high push-pull configuration established by sensor initialization. It replaces the register's other sources and verifies readback. Enable requires an active, healthy FIFO; disable remains available after a FIFO fault. A failed bus operation leaves routing uncertain and must be handled explicitly. FIFO start/stop preserves the route; sensor reset clears it. Register behavior follows [ST's datasheet, sections 9.10 and 9.15](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf).

The [`imu_fifo_irq`](../../samples/imu_fifo_irq/) example uses the Sense board's existing **INT1 → P0.02** connection, shown in the [Sense schematic](https://files.seeedstudio.com/wiki/XIAO_nRF54L15/Getting_Start/nRF54L15_Sense_Schematic.pdf) and installed board devicetree. No external interrupt wire is required. The upstream IMU driver stays disabled and the shared sensor/microphone supply stays enabled.

The GPIO callback wakes the application through `gpio_event.h`. All I2C transactions occur in thread context. Each wake drains at most 32 pairs at a nominal 416 Hz; if a backlog still holds INT1 active, the next wait returns immediately without needing another edge. The first pair remains discarded after FIFO start. Sample timestamps still identify host fetch completion, and neither callback counts nor wakes are acquisition timestamps.

Once per second the sample prints `IMUIRQ,samples=...,irqs=...,max_batch=...,fetch_us=...,accel_mg=x/y/z`. It checks that IRQ callbacks continue progressing, exits after a 100 ms wait expires or one second passes without complete samples, and ends on FIFO/bus errors. Cleanup disables sensor routing, stops the GPIO helper, and clears the FIFO, reporting each result. Static callback storage remains allocated if cleanup fails. Sensor conversion is not powered down. Physical INT1 activation and draining passed the release bench. Current consumption and deliberate stop/cleanup fault injection remain separate checks.

### IMU suspend and resume

`nrf54l15_lsm6ds3tr_suspend()` clears INT1 routing and FIFO, then sets both accelerometer and gyroscope ODR fields to zero while preserving their range selections. `resume()` restores the rate/ranges saved by initialization, waits 100 ms, and discards the first successful direct read to clear potentially stale output. Both transitions check chip identity and read back the sampling, FIFO-mode and interrupt-routing registers. ODR-zero power-down is specified in [ST's datasheet, sections 5.1 and 9.13–9.14](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf).

Any failed transition latches `POWER_FAULT` before callers can acquire more data. Direct motion/temperature reads, FIFO reads/start and INT1 enable then return `-EPIPE` without accessing the bus. Retain the object and explicitly retry suspend/resume, or reinitialize. FIFO stop and INT1 disable remain available for cleanup but do not repair the power state. A completed suspend rejects acquisition with `-EACCES`. Repeating the current healthy power state is a no-op; calling resume while already running does not clear an independent FIFO fault.

Resume leaves FIFO and INT1 disabled. Restart them explicitly and begin a new filter/timing history, because motion during suspension was not captured. Stop an attached GPIO event helper before changing power state and recreate its notification lifecycle when resuming IRQ acquisition. The `ready` field means initialization succeeded; acquisition also requires `power == RUNNING`. Keep exclusive ownership and the shared IMU/microphone supply enabled. These calls do not control that rail, the microphone or CPU sleep, and do not establish board-level current consumption. After an actual supply interruption, use initialization rather than resume.

The [`imu_power_cycle`](../../samples/imu_power_cycle/) example captures 104 paired samples at a requested 104 Hz, suspends the IMU for two seconds, then resumes and starts another capture window. It prints `IMUPOWER,epoch,AWAKE/SUSPENDED/RESUMED,...` milestones, checks that reads are blocked during suspension, and ends on a bus error or three-second capture timeout. On failure it makes one explicit suspend attempt and reports both status codes. The cycle includes capture, serial output and the resume settling interval; it is not an exact three-second timer. Repeated hardware suspend/resume passed the release bench, including the suspended-read rejection. Physical stale-temperature behavior and awake/suspended current measurements remain unverified.

## MCU and IMU temperature

`nrf54l15_temperature_sample` holds signed millidegrees Celsius and the host fetch-completion timestamp. Conversions round to the nearest millidegree, with exact ties away from zero. The extra decimal places describe the representation, not sensor accuracy. Both readers leave the entire output unchanged on failure and require serialized access in thread context.

For the MCU, enable `CONFIG_SENSOR=y`, `CONFIG_TEMP_NRF5=y` and `&temp { status = "okay"; };`, then call:

```c
const struct device *die = DEVICE_DT_GET(DT_NODELABEL(temp));
struct nrf54l15_temperature_sample temperature;
int err = nrf54l15_die_temperature_read(die, &temperature);
```

The nRF54L15 [TEMP peripheral](https://docs.nordicsemi.com/r/bundle/ps_nrf54l15/page/temp.html) reports chip temperature in 0.25 °C steps. When MPSL is enabled, NCS v3.3.0 selects its alternate temperature backend. This adapter takes NCS's shared `multithreading_lock` across sensor fetch/get so it cannot run concurrently with low-priority MPSL processing. Contention returns `-EBUSY` without waiting; retry later. Other code using this device must follow the same serialization and MPSL ownership rules. The reviewed SDK implementations are `nrf/drivers/mpsl/temp_nrf5/temp_nrf5_mpsl.c`, `nrf/subsys/mpsl/init/mpsl_init.c` and `nrfxlib/mpsl/doc/mpsl.rst`.

The API cannot enforce a conversion deadline: the non-MPSL SDK driver waits indefinitely for its clock/conversion completion and uses assertions for clock-request failures; MPSL's temperature API also blocks until measurement completes. Fetch/get errors are propagated when the backend returns them. Use a worker if the main acquisition loop has timing deadlines, and do not treat the host timestamp as a hardware acquisition timestamp.

For the direct IMU driver, call `nrf54l15_lsm6ds3tr_temperature_read()` after initialization. It checks `STATUS_REG.TDA`, reads the two-byte signed temperature output, and converts the nominal `25 + raw / 256` °C scale. The first successful burst after init/resume is discarded; absent data and that discard return `-EAGAIN`. Temperature has its own discard state and does not consume motion output or FIFO words. It remains readable after a FIFO fault, but does not repair the fault; suspended or failed power states still block it. ST specifies a temperature offset of up to ±15 °C in [datasheet table 5](https://www.st.com/resource/en/datasheet/lsm6ds3tr-c.pdf). Use it for characterized temperature trends; neither sensor establishes ambient temperature or supplies gyro temperature compensation automatically.

The [`temperature_poll`](../../samples/temperature_poll/) example reads both chips and prints `TEMP,die_fetch_us,die_mC,imu_fetch_us,imu_mC`, then sleeps one second. It keeps the shared sensor supply powered and disables the upstream IMU node for exclusive direct access. The two fetch timestamps are independent. No-data/lock-contention retries are limited to 200 attempts with 5 ms sleeps; this does not bound a blocking SDK call. A returned error ends the sample with an IMU suspend attempt. Optional `mpsl.conf` exercises the alternate MCU backend; it initializes MPSL but starts no radio link. Physical accuracy, stale-value behavior across resume and operation during radio traffic remain unverified.

## GPIO activation notifications

`gpio_event.h` wraps a [Zephyr GPIO callback](https://docs.zephyrproject.org/latest/hardware/peripherals/gpio.html) and a binary semaphore. Initialize its state to zero and pass a valid `gpio_dt_spec` for an exclusively owned pin. The helper validates the controller's supported-pin mask, accepts active-high/low polarity and one optional pull, and rejects other flags. It configures input and edge-to-active notification. Use normal kernel-aware IRQs; this adapter depends on single-CPU interrupt exclusion and rejects SMP builds.

`wait(timeout_ms)` first consumes a pending notification, then checks the logical pin level before sleeping. It returns immediately if the pin is already active, preventing an asserted sensor line from being overlooked just because its edge preceded the wait. Otherwise it waits up to the requested 0–`INT32_MAX` milliseconds. An empty zero-time poll returns `-EBUSY`; an expired timed wait returns `-EAGAIN`. GPIO read failures propagate.

Notifications coalesce: multiple callbacks can occupy one semaphore slot. A wake can also be stale after another operation cleared the source. Always inspect/drain the peripheral after waking. The saturating `activations` statistic counts callbacks observed by the MCU, not samples or guaranteed physical edges. A level-only wake does not increment it. This helper does not debounce inputs or configure system-off wake; use `button.h` for debounced button polling.

Keep the state alive and unmoved until `stop()` succeeds, including after a failed start. A callback can be registered before interrupt arming fails. Stop marks it inactive, disables the interrupt, requests input reconfiguration to release the nrfx GPIOTE channel, removes the callback under interrupt exclusion, and clears the semaphore. On failure retain storage and retry. Public operations, including wait and stop, must be serialized in thread context; use finite waits rather than stopping from a second thread.

An SDK limitation remains: NCS v3.3.0 `gpio_nrfx_pin_configure()` returns its initial `ret` value on several lower-level `err` paths, so some pin-configuration failures can appear successful. The wrapper propagates errors that the SDK returns but cannot prove physical pin configuration or channel release from that status alone. The native tests model explicit failures, callback lifetime, wake races and shared-port behavior; they do not validate GPIOTE electrical behavior or real Zephyr semaphore scheduling.

## OpenFloat integration

The example defers the SDK IMU device initialization until `main()` has waited 100 ms for the shared sensor supply to settle, then checks `device_init()` before configuring the wrapper. This prevents an early first-boot probe from leaving the sensor unavailable for the whole boot. An initialization error still ends acquisition explicitly.

`openfloat_telemetry` requests 416 Hz using absolute tick deadlines. The periodic helper distributes rounding across deadlines: at the current 31,250 Hz kernel clock, intervals alternate between 75 and 76 ticks instead of rounding every period to 76 ticks (about 411 Hz). Sensor I/O and filter work execute in the sampling thread. Every eighth scheduled sample is copied into a bounded eight-entry queue for a separate serial/BLE worker. Enqueue never waits. If the worker falls behind, new telemetry is dropped and counted; event records can also be dropped, while the cumulative shot count remains in later live frames.

`# OFSTATS,1,...` reports queue drops, missed sampling slots (`missed_ticks`), sample errors surfaced by the backend, and BLE send errors. Sequence numbers advance over missed scheduled samples. These counters make overload visible; actual sample rate and bus timing still require hardware measurement. Timestamps identify host fetch completion, not the IMU's internal sampling instant. Polling does not guarantee capture of every hardware sample.

The BLE service keeps subscription separate from start/stop state, passes notification peer selection to Zephyr, and schedules advertising after the connection is recycled. Initialization can retry a failed advertisement without enabling the stack again; an already active advertiser is success. Call initialization from one thread before connecting. The zero command posts an atomic request consumed by the sampling thread. Notification sends may wait for stack buffers, which is why they run in the transport worker. These choices follow the installed SDK interfaces and [Zephyr's connection](https://docs.zephyrproject.org/latest/services/connectivity/bluetooth/api/connection_mgmt.html) and [GATT](https://docs.zephyrproject.org/latest/services/connectivity/bluetooth/api/gatt.html) documentation.

The existing wire format remains:

```text
OFRAW,1,sequence,uptime_us,dt_us,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps,roll_cdeg,pitch_cdeg,yaw_cdeg,qw_1e6,qx_1e6,qy_1e6,qz_1e6,shot_count
OFSHOT,1,shot_id,uptime_us,ax_mg,ay_mg,az_mg,shot_count
```

The binary live frame is exactly 20 bytes:

| Offset | Value |
| --- | --- |
| 0–1 | ASCII `OF` |
| 2–3 | Version 1, type 1 |
| 4–5 | Sequence modulo 65536, little endian |
| 6–7 | Sample interval in microseconds, saturated to 65535 |
| 8–13 | Three signed little-endian int16 accelerations in mg |
| 14–19 | Three signed little-endian int16 angular rates in degrees/s × 16 |

There are no calibration flags or checksum in this version. Encoders saturate measurements and reject nonfinite inputs. Serial angles are zeroed by roll/pitch offsets; quaternions remain the unzeroed orientation. Yaw has no magnetic reference and can drift. The impact detector reports threshold candidates; its thresholds need field validation before interpreting events as confirmed shots.

[`pc-app/openfloat_protocol.py`](../../pc-app/openfloat_protocol.py) provides dependency-free Python 3.10+ decoders and a sequence-gap helper that accounts for decimation, wrap, and ambiguous resets.

[`pc-app/verify_openfloat.py`](../../pc-app/verify_openfloat.py) checks live serial data or a raw capture without the web app. It reports malformed frames, missing notifications, resets/duplicates, quaternion norm, sample intervals, received frame rate, and the latest `# OFSTATS` device counters. Those counters are cumulative since the device's latest reset, modulo 2³², rather than loss totals for the capture interval. Missing counters are reported as `null`, not zero. Serial chunks are reassembled across read timeouts; an unfinished final line is omitted. Captures are written to new files only. Exit status is 1 for too few frames, malformed frames, or nonunit quaternions; gaps, discontinuities, and device counters are reported separately.

```powershell
# Replace COM11 with the current XIAO port. The client asserts DTR and RTS.
python pc-app/verify_openfloat.py `
  --port COM11 --seconds 10 --capture build/capture.log
python pc-app/verify_openfloat.py --input build/capture.log
```

If the VCOM is silent, keep it open and reset the target as described in [hardware testing](../../docs/hardware-testing.md). The verifier does not reset or flash devices itself.

## ADC acquisition

The ADC adapter reads one configured channel synchronously. It calibrates before the first successful reading; passing `calibrate=true` requests another calibration. If a read that requests calibration fails, calibration remains pending for retry. Raw and converted values are published together only after every operation succeeds. Millivolts describe the ADC input; an external resistor divider still requires application scaling. Missing reference-voltage information is rejected instead of yielding a misleading zero-voltage conversion.

The [`adc_poll`](../../samples/adc_poll/) example uses the NCS board's AIN0 channel at XIAO D0/A0 (P1.04), with its configured gain, internal reference, resolution and oversampling. It prints `ADC,uptime_us,raw,input_millivolts`. Offset calibration, conversion, and error behavior are unit-tested; physical voltage accuracy remains unverified. Supported input limits are set by VDD, not solely the gain/reference setting; see [Nordic's SAADC reference description](https://docs.nordicsemi.com/r/bundle/ps_nrf54l15/page/saadc.html-padc_reference).

## Stationary gyro calibration

The calibration helper estimates a constant gyro bias from a caller-selected window of uniformly spaced, stationary samples. It checks acceleration magnitude near gravity, angular-rate magnitude, and the combined XYZ sample standard deviations of both sensors. Welford accumulation avoids subtracting nearly equal squared sums. Motion or nonfinite measurements discard the pending window. A completed result stays fixed until the next `begin`; the caller owns and subtracts the bias in radians/s from future gyro samples.

Call `begin` again after a missing sample or transport error. The helper has no clock and cannot identify gaps by itself. Slow constant yaw rotation cannot be distinguished from a constant gyro bias using only these six axes, so the operator must hold the board still. The estimate does not compensate changing temperature, scale errors, or magnetically unreferenced yaw.

The [`imu_calibration`](../../samples/imu_calibration/) sample uses the direct I2C driver at 104 Hz, ±4 g and ±250 degrees/s. Put the board on a stable surface before resetting it. After a one-second pause, it requires 208 acceptable samples and aborts after 30 seconds. Gaps exceeding 20 ms restart the window. Its example thresholds completed a stationary bench window; characterize them for your mounting, temperature and motion environment. Successful calibration prints `GBIAS,x_mdps,y_mdps,z_mdps`, then `GCORR` records containing raw gyro, corrected gyro, and the corrected filter's Euler angles. All axes are the sensor's native axes; bias is stored only in RAM and reset starts calibration again. This sample uses separate record names; the active OpenFloat application's raw telemetry and zero command retain their existing meaning.

### Persistent gyro bias

The optional `gyro_bias` codec stores a 28-byte record containing a format version, application-assigned profile ID, three signed bias components in microradians/s, and a CRC-32/ISO-HDLC checksum. The fixed-point encoding is independent of C structure layout and byte order. It rounds each component to the nearest microradian/s and rejects nonfinite or unrepresentable values. Encoding and decoding leave output untouched on failure. The checksum detects accidental corruption; it does not authenticate a calibration or prove that the board was stationary when it was measured.

| Bytes | Field |
| --- | --- |
| 0–3 | ASCII `GCAL` |
| 4 | Version 1 |
| 5–7 | Reserved, zero |
| 8–11 | Profile ID, little-endian uint32 |
| 12–23 | Native-axis X/Y/Z bias, little-endian int32 microradians/s |
| 24–27 | CRC over bytes 0–23, little-endian uint32 |

The Settings adapter stores one record under `n54/gyro`. Its load/save/clear calls initialize the subsystem as needed and report errors returned by the backend. Loading checks exact length, checksum, version, reserved bytes and profile before publishing a bias. A profile mismatch returns `-EXDEV`; an unknown format version returns `-EPROTONOSUPPORT`. The application assigns a new profile when a change to sensor configuration or axis convention makes its previous calibration inapplicable. The codec does not decide whether a bias is physically plausible or still appropriate for the current temperature.

Use Settings with **ZMS** on the nRF54L15's RRAM, following the installed NCS guide at `nrf/doc/nrf/app_dev/device_guides/nrf54l/zms.rst` and [Zephyr's ZMS guidance](https://docs.zephyrproject.org/latest/services/storage/zms/zms.html). The example enables both ZMS data CRC and duplicate-write suppression. Storage operations can block and belong outside the acquisition loop. `clear()` deletes only this key; the adapter has no partition-format or automatic-repair operation. A failed save is not proof that the old value survived; reload and validate before using stored data afterward.

Build `imu_calibration` with `persistent.conf` to restore a bias on reset. **Hold the user button during reset to measure and save a bias, including the first calibration.** Release the button and put the board on a stable surface when prompted. Without that request, missing or invalid data ends the example with a diagnostic; it does not overwrite the record. This also addresses an SDK limitation: NCS v3.3.0's `settings_zms_find_hash_from_name()` discards name-read failures, so some storage faults can appear as `-ENOENT` to this adapter. The sample uses profile `0x4c530001` for its native-axis LSM6DS3TR-C configuration. Reset restores the saved bias but starts a new orientation estimate.

The optional build reserves an 8 KiB Settings partition through NCS Partition Manager. Its verified generated layout puts application space at `0x000000–0x162FFF` and Settings at `0x163000–0x164FFF`. Verify the generated `partitions.yml` before integrating persistent data into a different application. Keeping a calibration across reflashes also requires a stable partition/backend configuration and a programming process that preserves that region. Save, software-reboot retention, reload, duplicate save and clear passed a controlled bench. Power-removal retention, interrupted writes and storage endurance remain unverified.

## PWM output

The PWM adapter copies a `pwm_dt_spec` and requests an inactive output at initialization. Set active time with `nrf54l15_pwm_set_pulse_ns()`, or call `nrf54l15_pwm_set_duty_permille()` with 0–1000, where 250 means 25%. Duty conversion rounds to the nearest nanosecond without overflowing a 32-bit period. The stored period and polarity remain fixed. With inverted polarity, active time is the low portion of the waveform.

```c
static const struct pwm_dt_spec pin = PWM_DT_SPEC_GET(DT_PATH(zephyr_user));
struct nrf54l15_pwm output;
int err = nrf54l15_pwm_init(&output, &pin);
if (!err) {
    err = nrf54l15_pwm_set_duty_permille(&output, 250);
}
/* Handle err. Request duty 0 when the application needs an inactive output. */
```

All four channels on one Nordic PWM controller share its period. Serialize controller access and use matching periods for simultaneous waveforms; the installed driver rejects incompatible periods with `-EINVAL`. Initializing to inactive does not prove that a nonconstant waveform at the requested frequency is supported. Driver errors propagate, and no cached value is presented as physical output readback. The SDK further quantizes nanoseconds to clock cycles and a prescaler; very short pulses can round to zero. Restarting a stopped PWM can wait for the previous period to finish, so use thread context outside timing-critical acquisition loops. These semantics follow the installed `pwm_nrfx.c` and [Zephyr's PWM API](https://docs.zephyrproject.org/latest/hardware/peripherals/pwm.html).

The [`pwm_output`](../../samples/pwm_output/) example uses PWM20 channel 0 on XIAO D0/A0 (P1.04), nominally 1 kHz. Its duty rises and falls over approximately two seconds. Observe D0 against GND with a scope, or connect D0 through a 1 kΩ resistor to an external LED's anode, with its cathode connected to GND. This sample disables the ADC, which shares that pin in `adc_poll`. It prints occasional `PWM,duty_permille` records. The built-in XIAO LED on P2.00 cannot use these hardware PWM controllers: [Nordic's pin-planning guidance](https://devzone.nordicsemi.com/nordic/nordic-blog/b/blog/posts/essential-pin-planning-guidelines-for-the-nrf54l-series) restricts PWM20/21/22 to P1. Pin routing, waveform frequency, and duty accuracy still require board measurements.

## Watchdog progress channels

The watchdog adapter starts one to eight hardware reload channels with a common timeout and SoC-reset behavior. Give each critical worker a logical slot and call `nrf54l15_watchdog_feed()` for that slot after useful work completes. The library never feeds automatically. [Nordic's reload criteria](https://docs.nordicsemi.com/r/bundle/ps_nrf54l15/page/wdt.html-concept_d5y_nzw_sr) require all enabled channels to be fed before the shared counter reloads. Consequently, these are not independent deadlines measured from each worker's last feed: staggered feeds can make the delay from one worker's last progress to expiry approach twice the configured period.

```c
static struct nrf54l15_watchdog watchdog; /* Must initially be zero. */
int err = nrf54l15_watchdog_start(&watchdog,
    DEVICE_DT_GET(DT_ALIAS(watchdog0)), 2000, 2, WDT_OPT_PAUSE_HALTED_BY_DBG);
/* Check err before starting workers. Each worker owns one slot: */
/* worker 0: err = nrf54l15_watchdog_feed(&watchdog, 0); */
/* worker 1: err = nrf54l15_watchdog_feed(&watchdog, 1); */
```

The device must belong exclusively to this adapter from boot. Serialize start/stop with all feeds; workers can feed different slots concurrently once startup is complete. Inputs are checked before any channels are installed. A later startup failure latches `FAULTED`, because Zephyr cannot uninstall a partly configured set before starting the watchdog. Reset the device before retrying such a failure; clearing the C structure alone does not clear the driver or hardware. Feed errors propagate without pretending that progress was accepted.

`nrf54l15_watchdog_stop()` is an explicit operation. The nRF54L15 SDK enables hardware stop support; a successful disable releases channels and permits a fresh start. A failed stop retains the adapter's configuration and does not establish that the timer stopped. The adapter follows the installed `wdt_nrfx.c` and [Zephyr's watchdog interface](https://docs.zephyrproject.org/latest/hardware/peripherals/watchdog.html). Sleep and debugger pause options are explicit; pausing in sleep changes the relationship between elapsed wall time and timeout.

The [`watchdog_recovery`](../../samples/watchdog_recovery/) example enables the XIAO's `watchdog0` alias, WDT31, with two channels and a 2-second timeout. It feeds both simulated workers eight times, then withholds slot 1 while continuing to feed slot 0. It intentionally causes a watchdog reset when run on a board. On the following boot it prints the reset-cause mask and, if `RESET_WATCHDOG` is present, ends without rearming. It reads and clears reset causes through Zephyr's hardware-info API and refuses to arm if either operation fails. A manual reset starts another demonstration. Debugger halts pause this example's timer; ordinary kernel sleep does not. The release bench observed the reset and RESET_WATCHDOG cause after withholding one slot. Exact elapsed timeout and stop/restart timing were not measured.

## Synchronous SPI transactions

`spi.h` provides 8-bit full-duplex controller transactions through the [Zephyr SPI API](https://docs.zephyrproject.org/latest/hardware/peripherals/spi.html). It accepts CPOL/CPHA modes 0–3, MSB/LSB bit order, and GPIO chip select or no chip select. The adapter accepts requests from 125 kHz to 32 MHz, but the backend may reduce or reject a request that its controller cannot represent. XIAO SPIM00 rejects 1 MHz with -EINVAL in NCS v3.3.0; the example requests 2 MHz, which passed wire loopback. GPIO CS must be declared in the bus devicetree so the SDK configures it, and its active level must match the SPI flags. Native CS timing, explicit interword delay, half-duplex, multiline, internal loopback, TI framing, CS hold and bus locking are rejected instead of silently ignored.

Zero-initialize each `nrf54l15_spi` in persistent storage. Initialization checks/copies configuration and performs no wire transaction; actual controller configuration happens during transfer. **Do not modify, move, copy or clear the initialized instance.** NCS v3.3.0 `spi_nrfx_spim.c` caches configuration by pointer identity, so an object's address must continue to identify the same configuration. Reinitialization returns `-EALREADY`. Use distinct persistent instances for different devices or modes. Serialize calls to an instance; the SDK serializes the shared controller between transactions.

`nrf54l15_spi_transfer()` handles one complete frame of 1–512 bytes. Assemble command, address, dummy clocks and payload in the transmitted bytes as required by the peripheral's protocol. A null TX sends the supplied dummy byte repeatedly; a null RX discards incoming data. Both may be null to generate clocks only. The adapter uses two 512-byte aligned RAM buffers per instance, allowing read-only/unaligned TX and unaligned or overlapping caller TX/RX. Caller buffers must not point inside the instance. RX is copied back only after SDK success, including when caller TX and RX are the same buffer.

Errors propagate without automatic retries. Preserving RX on error does not undo a peripheral write: a failed transaction may already have sent some or all bytes. The SDK supplies transfer completion timeout and abort handling, while acquisition of the shared bus lock can wait indefinitely. A successful transfer is not proof that the intended peripheral responded; verify response bytes using that peripheral's protocol. In the installed SDK, `spi_context.h` also ignores GPIO CS set errors, so this wrapper cannot report those hidden failures.

The [`spi_loopback`](../../samples/spi_loopback/) sample tests the board's SPIM00 pins using an **external wire from D10/SDO (MOSI, P2.02) to D9/SDI (MISO, P2.04)**. D8/P2.01 supplies clock; D0/P1.04 is configured as active-low CS and may be left unconnected or observed with a logic analyzer. Disconnect peripherals from these pins before running the pattern generator. These pin assignments come from the installed XIAO board's connector and pinctrl files.

Four persistent configurations exercise modes 0–3 at a requested 2 MHz, sending varying byte patterns at lengths 1, 3, 16, 255, 256 and 512. It prints `SPI,round,mode,bytes,PASS` after each matching transfer and ends on the first driver error or mismatch. Its RAM includes four adapter instances and two 512-byte application buffers. An external wire test checks the local data path; it does not validate a peripheral's timing requirements, CS waveform or operation at the maximum SPI frequency. Modes 0–3 and all listed lengths passed the release wire-loopback bench. CS waveforms, external-device timing and maximum frequency remain unmeasured.

## Buffered UART reception

`uart_rx.h` receives bytes into caller-provided storage through the [Zephyr interrupt-driven UART API](https://docs.zephyrproject.org/latest/hardware/peripherals/uart.html). Zero-initialize the receiver and supply 1–32767 bytes of storage. Both must remain alive and unmoved until stop succeeds. The adapter exclusively owns the device's IRQ callback and RX/error notifications; select the corresponding `CONFIG_UART_<instance>_INTERRUPT_DRIVEN` option and keep async UART, console, shell and interrupt-driven TX off that device. Polling TX with `uart_poll_out()` is supported. Baud and framing come from devicetree.

On the XIAO, UART21 crosses into the P2 pin domain. The examples enable `CONFIG_NRF_SYS_EVENT=y` and `CONFIG_SOC_NRF_FORCE_CONSTLAT=y`, as required by [Nordic's dedicated-pin guidance](https://docs.nordicsemi.com/r/bundle/ps_nrf54l15/page/chapters/pin.html-dedicated_pins?contentId=ckWYM65jMv4DBKTI__JinQ). Omitting Constant Latency produced framing/break errors on the bench. This mode has a power cost; the application owns that power policy.

Public calls run serially in thread context. `read()` is nonblocking and copies at most 256 bytes per call. It uses single-CPU interrupt exclusion to protect the queue and loss state; SMP is deliberately unsupported. No user callback runs in the IRQ. Receive overflow or a UART line error latches `-EOVERFLOW`, preserves caller output and discards subsequent received bytes. Call `resync()` to discard the software backlog and acknowledge the gap, then reset the application parser and seek its next wire delimiter/header. This operation alone does not establish a message boundary.

Negative IRQ/FIFO API failures latch the first fault and disable notifications; stop/start is required to recover. `stop()` disables notifications, removes the callback and discards queued bytes. If callback removal fails, retain the state/storage and retry stop. The installed nrfx backend can retain a hardware byte when RX notifications are disabled, so neither stop nor start promises a clean hardware FIFO. Successful stop is a memory-lifetime boundary, not a guarantee of physical UART power-down.

Diagnostics count received bytes, software discards, overflow events and UART error events, and retain the OR of line-error bits. Counters saturate at `UINT32_MAX`. A software discard includes queued bytes thrown away by resync/stop; hardware-lost bytes cannot be counted from the available error flags. Running state describes the software request, not physical readback.

The [`uart_loopback`](../../samples/uart_loopback/) sample uses an **external wire from D6/TX/P2.08 to D7/RX/P2.07**, at 115200 8N1 on UART21. UART20 remains the USB debug console. Each round checks a 64-byte pattern, intentionally sends 192 bytes into a 128-byte queue without reading, verifies the loss latch, acknowledges the gap, then checks another 64-byte pattern. It prints cumulative `UART,round,PASS,received=...,discarded=...,overflows=...` diagnostics or ends on an error. The release bench passed matching patterns, intentional overflow and explicit recovery. Injected electrical line faults and long-duration sustained capacity remain unmeasured.

## Binary packets over a byte stream

`packet.h` adds an optional envelope for arbitrary 0–256-byte payloads:

```text
0x00 | COBS(payload || CRC32_LE(payload)) | 0x00
```

[COBS, described by Cheshire and Baker](https://www.stuartcheshire.org/papers/COBSforToN.pdf), removes zero bytes from the encoded body so a zero can mark its boundary. This library sends both a leading and trailing zero for every packet; repeated zeros are harmless. Its wire representation occupies 7–264 bytes, including the CRC and delimiters. Payload types, versions and sequence numbers are application data. It does not add acknowledgments, retransmission or authentication, and it does not change the active OpenFloat transport.

`nrf54l15_packet_encode()` returns the actual wire length through `written`, accepts overlapping payload/output buffers and preserves output on error. `nrf54l15_packet_reset()` initializes the stream decoder and puts it into discard-until-zero mode. Feed bytes in order with `nrf54l15_packet_feed()`: return 1 means a complete CRC-checked payload, including a valid empty payload; 0 means no completed packet; negative errno reports a discarded malformed/oversized packet or insufficient output capacity. Output and its length change only on return 1. Decoder state and caller output must not overlap.

The decoder stores at most 262 encoded bytes. An overlong nonzero run reports one size error and is discarded through its delimiter. A partial packet can remain buffered indefinitely; applications that require a timeout call reset when their own deadline expires. **After UART reports `-EOVERFLOW`, reset the packet decoder as well as acknowledging the UART gap with `uart_rx_resync()`.** Resynchronization discards an incomplete fragment and resumes at a later boundary; it cannot recover missing packets. Keep application sequence numbers if missing whole packets must be detected.

`crc32.h` exposes the checksum already used by gyro calibration records: reflected polynomial `0xEDB88320`, initial state and final XOR `0xFFFFFFFF`. `"123456789"` produces `0xCBF43926`; empty input produces zero. Existing calibration record bytes remain unchanged. Encoding/decoding use bounded stack workspaces; the caller owns persistent decoder state and output arrays.

The [`uart_packet_echo`](../../samples/uart_packet_echo/) sample combines the packet decoder with a 512-byte UART receive queue. Connect a **3.3 V UART adapter**: adapter RX to XIAO D6/TX/P2.08, adapter TX to D7/RX/P2.07, and common GND. Use UART21 at 115200 8N1; the board's USB debug console stays on UART20 and prints `PACKET,echoed=...,rejected=...,gaps=...,discarded=...` once per second. The sample echoes only valid payloads, resets its parser on receive loss, and stops on a backend error. Send one request and wait for its echo; polling TX can delay receive consumption during larger replies.

The independent Python reference in [`packet_protocol.py`](../../pc-app/packet_protocol.py) uses `zlib.crc32` and accepts arbitrary read chunk sizes. With the echo sample running, use [`uart_packet_probe.py`](../../pc-app/uart_packet_probe.py) against the **external adapter's port**, replacing `COM12` below with its actual port:

```powershell
python pc-app/uart_packet_probe.py --port COM12 --rounds 257 --timeout 1
```

Live probing requires `pyserial`. The default 257 rounds cover every payload size and include 16 rejection/recovery probes cycling through bad CRC, truncated and oversized frames. The probe checks exact echoes, malformed/duplicate replies, partial writes and deadlines; it does not flash or reset a board. Native/Python tests validate the codecs and a simulated peer. A local D6→D7 bench passed all payload lengths and 16 malformed-frame rejection/recovery probes through the real UART. The independent external-adapter echo path and sustained traffic remain unverified.

## Switched battery-voltage measurement

The [`battery_poll`](../../samples/battery_poll/) sample reads the onboard battery divider. The [XIAO Sense schematic, power sheet](https://files.seeedstudio.com/wiki/XIAO_nRF54L15/Getting_Start/nRF54L15_Sense_Schematic.pdf) connects P1.15 to the TPS22916 divider-enable input and P1.14/AIN7 to the midpoint of two 10 kOhm resistors, giving a nominal 2:1 voltage scale. This switch controls the measurement circuit, not the board's main supply. The sample removes `regulator-boot-on` from the SDK's `vbat_pwr` node so its regulator starts with no power reference held.

`nrf54l15_battery_init()` takes the ADC devicetree specification, the divider regulator, resistance ratio and settling delay. It rejects an already-enabled regulator instead of consuming another client's reference. Own this regulator exclusively and serialize both adapter calls and use of the ADC. Initialize the state to zero; configuration is copied, and an already-initialized instance rejects reinitialization.

Each read enables the regulator, waits the configured settling time, obtains an ADC sample, and releases its power reference even when the ADC operation fails. The first successful ADC read calibrates its offset; callers can request recalibration later. The sample waits 100 ms per read and requests calibration again every 60 measurements. It sleeps one second after each result, so its period includes measurement/settling time rather than being exactly 1 Hz.

On a release failure, `power_held` remains true and subsequent reads return `-EBUSY`. Call `nrf54l15_battery_release()` to retry the pending release; do not clear the object to hide it. A release error takes precedence over an ADC error. A failed enable is returned without a corresponding disable because the installed NCS common regulator backend rolls back that reference itself. Successful disable releases only the adapter's reference. An unexpected remaining reference causes a read to fail without decrementing another owner's count. These are software ownership checks, not physical rail-voltage readback.

Output is published only after acquisition, release and divider scaling all succeed. `sample.input` retains raw counts, ADC-input millivolts and the conversion-completion timestamp. `sample.millivolts` applies the resistance ratio using checked 64-bit arithmetic and rounds to the nearest integer millivolt, with half values away from zero. Out-of-range results return `-ERANGE`; small negative ADC readings stay signed. Resistor/reference tolerances and the ADC helper's integer-millivolt conversion still limit accuracy.

Serial output is `BATTERY,uptime_us,raw,input_mv,battery_mv`. It reports voltage only: it does not identify whether a battery is attached, determine charging state, protect a cell, or estimate remaining charge. Compare it against a meter before using the result in an application.

The default build keeps the USB UART console for bench checks. The optional `standalone.conf` disables UART and selects nonblocking RTT output, following [Seeed's hardware-v1.0 battery-boot guidance](https://wiki.seeedstudio.com/xiao_nrf54l15_sense_getting_started/#battery-voltage-detection). RTT requires a compatible attached debug probe to observe the output. This variant does not establish low-power performance: other board rails and devices retain their defaults, and current consumption and battery-only boot remain hardware checks.

## PDM microphone and PCM levels

The [`microphone_levels`](../../samples/microphone_levels/) sample uses the Sense board's onboard PDM microphone through Zephyr's DMIC API. The installed NCS v3.3.0 board definition connects PDM20 clock to P1.12 and data to P1.13, with shared microphone/IMU power enabled by P0.01. Its overlay keeps that supply on and uses the board's `PCLK32M` clock source. Mono left-channel selection, requested 16 kHz PCM and microphone clock limits follow [Seeed's microphone example](https://wiki.seeedstudio.com/xiao_nrf54l15_sense_built_in_sensor/). The adapter handles PCM blocks according to the [Zephyr DMIC interface](https://docs.zephyrproject.org/latest/hardware/peripherals/audio/dmic.html).

`nrf54l15_dmic_start()` accepts a requested 8–48 kHz rate, PDM clock limits, left/right channel and an even sample count of 2–32766. It checks the negotiated channel mapping before starting. Rate bounds also avoid unsafe high-rate prescaler arithmetic in the installed nrfx backend. The backend chooses realizable clock divisors and does not report the actual PCM rate through this API. Treat the requested rate and sample-derived times as nominal; `PCLK32M` oscillator accuracy also matters.

Provide a dedicated, initialized `k_mem_slab` with at least four blocks, each exactly twice the configured sample count in bytes and at least four-byte aligned. Slab geometry validation uses the NCS v3.3.0 `k_mem_slab.info` layout. State, slab and backing memory must remain alive until stop succeeds; use static storage in simple applications. Own the peripheral and pool exclusively, and serialize all adapter calls in thread context.

`nrf54l15_dmic_read()` copies one complete block into the caller's PCM array and returns the DMA block to the pool before returning. Capacity is in samples. It preserves output on any reported failure, frees successfully received blocks even when their length is wrong, and accepts finite millisecond timeouts or zero for a nonblocking read. A timeout does not establish whether capture is still running: the SDK can stop itself after a queue/buffer overrun, and it exposes no discontinuity counter through DMIC.

**STOP is asynchronous in NCS v3.3.0 and leaves completed blocks queued.** `nrf54l15_dmic_stop()` discards that backlog and waits for all DMA buffers to return. It then disables the stream with a zero-rate configuration; the SDK's `-EBUSY` response provides an additional idle check when no buffers are allocated yet. Only successful cleanup returns the adapter to `IDLE` and permits another start. Any cleanup failure leaves `STOPPING`: keep all storage alive and retry stop. Do not zero the object or reuse the pool to force a restart. These lifecycle rules are based on the installed `zephyr/drivers/audio/dmic_nrfx_pdm.c`; other DMIC backends have not been validated.

The sample uses four 640-byte DMA blocks plus one 640-byte caller buffer. Each session captures 500 blocks after discarding five startup blocks, nominally 10 seconds after 100 ms settling, then stops and restarts after a one-second pause. This exercises repeated cleanup without raw audio transport. A capture/cleanup error ends the demonstration and prints both status codes. The static pool remains allocated even if cleanup fails.

It prints `MIC,session,block,peak,rms,mean,rail_samples` for every fifth retained 20 ms block. Each line describes that block, not the whole 100 ms interval. `pcm16.h` computes absolute peak safely for `INT16_MIN`, mean and RMS including DC, and the number of samples exactly at either signed PCM rail. RMS is rounded and mean truncated for integer serial output. Values are PCM code units, not calibrated sound-pressure levels; rail samples alone do not prove physical microphone clipping. The helper uses 64-bit sums and squared sums, with float precision in its final mean/RMS results.

## Build and test

Run these commands from the repository root with CMake, a native C compiler and Python available:

```sh
cmake -S . -B build/native
cmake --build build/native --config Debug
ctest --test-dir build/native -C Debug --output-on-failure
python -m unittest discover -s tests/python -v
```

See [testing](../../docs/testing.md) for the 18 native suites, 24 Python tests, 1,028 independent C/Python packet vectors and platform-specific executable selection. Tests use the production C code with hardware/OS doubles; they do not simulate silicon timing or radio coexistence.

The [example index](../../samples/README.md) lists all firmware build commands and configurations. The [verification record](../../docs/verification.md) records release-source builds, functional Sense-board tests, the fixes found on hardware and the remaining instrument-dependent measurements.
