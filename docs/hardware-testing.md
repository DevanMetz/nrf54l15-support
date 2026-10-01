# Hardware testing

Use a XIAO nRF54L15 Sense, a USB data cable, and a working CMSIS-DAP/debug-UART connection. Identify the current port and probe serial; do not assume an old COM number belongs to this board. If Windows reports "Device Descriptor Request Failed", fix USB enumeration before flashing.

## Console and flashing

Open the console at 115200 baud with DTR and RTS asserted. If startup output is silent, reset the target while the console remains open. Only one process can own a serial port.

The board's user button is printed **B** on the tested board; the SDK names it `USR`/`sw0` on P0.00. **R** is reset. SPI requires only D10→D9 and requests 2 MHz. UART21 requires D6→D7 for loopback, plus `CONFIG_NRF_SYS_EVENT=y` and `CONFIG_SOC_NRF_FORCE_CONSTLAT=y` for its P2 pins. See the API guide for the power-policy implications.

Use the SDK's configured runner when supported. A Seeed OpenOCD support configuration can also load nRF54L RRAM:

```sh
openocd -s /path/to/seeed/board/support -s /path/to/openocd/scripts \
  -c "adapter serial YOUR_PROBE_SERIAL" -f /path/to/seeed/board/support/openocd.cfg \
  -c "source /absolute/path/to/nrf54l15-support/tools/nrf54l15_rram.tcl" \
  -c "init" -c "reset init" -c "nrf54l-load /absolute/path/to/build/merged.hex" \
  -c "rram_commit" -c "verify_image /absolute/path/to/build/merged.hex" \
  -c "reset run" -c "shutdown"
```

This command depends on a support file defining `nrf54l-load`; it is not a command available in every OpenOCD installation. Back up existing application/persistent data before a bench sequence. The library does not contain the SDK, proprietary radio binaries, debug tools or board packages.

**Commit the final write buffer and verify the image before reset.** The tested Seeed helper enabled RRAM buffering but omitted the final commit. Eight trailing bytes remained from the previous application and caused a BLE startup bus fault. The included Tcl helper polls READY with a bounded timeout, commits pending writes and returns RRAM to read-only mode, following [Nordic's RRAM programming procedure](https://docs.nordicsemi.com/r/bundle/nan_047/page/app/nan_production_programming/nrf54l/nrf54l_writing_data_secureapprotect_not_enabled.html). A successful `load_image` message alone does not confirm the final bytes. Inspect verification output and stop on any mismatch; the release bench also compared a complete SWD readback against every HEX byte.

## Expected checks

| Area | Expected observation |
| --- | --- |
| Direct IMU | Repeated `IMU` lines, progressing sequence/timestamps, plausible gravity and motion; no bus failures |
| FIFO | Progressing `IMUFIFO`/`FIFOSTATS`, healthy continuity epochs; characterize cadence and overrun separately |
| INT1 | `IMUIRQ` sample and activation counts advance; cleanup reports success |
| Suspend/resume | Repeated `AWAKE`, `SUSPENDED`, `RESUMED` milestones; acquisition blocked while suspended |
| Calibration | Stationary window completes; corrected gyro values approach zero; movement resets the window |
| Persistence | Explicit button-requested save, reset, then restored bias; compare the stored result |
| Temperature | `TEMP` lines with signed millidegrees and progressing fetch times; compare both SDK backends |
| Microphone | Nonzero PCM levels responding to sound; multiple sessions stop and restart successfully |
| Watchdog | Both slots initially feed, withholding one causes reset, next boot reports `RESET_WATCHDOG` and does not rearm |
| SPI | With D10→D9 installed, every mode/length prints `PASS`; check clock/CS waveforms with a scope separately |
| UART | With D6→D7 installed, ordinary bytes match, forced overflow occurs, recovery prints `PASS` |
| Packet echo | External UART probe covers all payload lengths and malformed-frame recovery; console reports stats |
| ADC/battery | Measurements agree with a known reference within characterized error; divider switch releases after each read |
| PWM | Scope or external LED demonstrates duty changes; measure frequency/polarity separately |
| OpenFloat | Serial verifier passes; BLE receives valid 20-byte frames and zero/start/stop/disconnect/reconnect behave correctly |

Register-level power-down does not establish board current consumption. Temperature readings describe dies and require characterization for ambient use. Impact thresholds produce candidates that require field validation. A passing API call alone is not a physical timing/accuracy measurement.

Record SDK/library revision, board variant, fixture, duration, counts/errors and limitations in the [verification record](verification.md). Keep raw captures local when they contain device identifiers; publish summarized observations.
