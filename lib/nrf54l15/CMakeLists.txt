# SPDX-License-Identifier: Apache-2.0
add_library(nrf54l15_core STATIC
  src/orientation.c
  src/telemetry.c
  src/debounce.c
  src/impact.c
  src/periodic.c
  src/gyro_calibration.c
  src/gyro_bias.c
  src/pcm16.c
  src/crc32.c
  src/packet.c
)
target_include_directories(nrf54l15_core PUBLIC include)
target_compile_features(nrf54l15_core PUBLIC c_std_99)

if(TARGET zephyr_interface)
  target_link_libraries(nrf54l15_core PUBLIC zephyr_interface)
  add_dependencies(nrf54l15_core zephyr_generated_headers)
  if(CONFIG_SENSOR OR CONFIG_GPIO OR CONFIG_I2C OR CONFIG_ADC OR CONFIG_PWM OR CONFIG_SETTINGS OR CONFIG_WATCHDOG OR CONFIG_AUDIO_DMIC OR CONFIG_SPI OR (CONFIG_UART_INTERRUPT_DRIVEN AND CONFIG_RING_BUFFER))
    add_library(nrf54l15_drivers STATIC)
    target_link_libraries(nrf54l15_drivers PUBLIC nrf54l15_core zephyr_interface)
    add_dependencies(nrf54l15_drivers zephyr_generated_headers)
    if(CONFIG_SENSOR)
      target_sources(nrf54l15_drivers PRIVATE src/imu.c src/temperature.c)
    endif()
    if(CONFIG_GPIO)
      target_sources(nrf54l15_drivers PRIVATE src/button.c src/gpio_event.c)
    endif()
    if(CONFIG_I2C)
      target_sources(nrf54l15_drivers PRIVATE src/lsm6ds3tr.c)
    endif()
    if(CONFIG_SPI)
      target_sources(nrf54l15_drivers PRIVATE src/spi.c)
    endif()
    if(CONFIG_UART_INTERRUPT_DRIVEN AND CONFIG_RING_BUFFER)
      target_sources(nrf54l15_drivers PRIVATE src/uart_rx.c)
    endif()
    if(CONFIG_ADC)
      target_sources(nrf54l15_drivers PRIVATE src/adc.c)
      if(CONFIG_REGULATOR)
        target_sources(nrf54l15_drivers PRIVATE src/battery.c)
      endif()
    endif()
    if(CONFIG_PWM)
      target_sources(nrf54l15_drivers PRIVATE src/pwm.c)
    endif()
    if(CONFIG_SETTINGS)
      target_sources(nrf54l15_drivers PRIVATE src/gyro_bias_settings.c)
    endif()
    if(CONFIG_WATCHDOG)
      target_sources(nrf54l15_drivers PRIVATE src/watchdog.c)
    endif()
    if(CONFIG_AUDIO_DMIC)
      target_sources(nrf54l15_drivers PRIVATE src/dmic.c)
    endif()
  endif()
  if(CONFIG_BT_PERIPHERAL)
    add_library(nrf54l15_openfloat_ble STATIC src/openfloat_ble.c)
    target_link_libraries(nrf54l15_openfloat_ble PUBLIC nrf54l15_core zephyr_interface)
    add_dependencies(nrf54l15_openfloat_ble zephyr_generated_headers)
  endif()
elseif(UNIX)
  target_link_libraries(nrf54l15_core PUBLIC m)
endif()
