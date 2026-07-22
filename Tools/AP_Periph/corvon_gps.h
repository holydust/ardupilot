/*
  CORVON G-series GNSS module support: connector-powered mode selection,
  RS2058 passthrough switch control and status LED patterns.

  The module has two host connectors (10-pin UART+I2C, 4-pin CAN). Which
  one is powered decides the interface mode, latched once at boot:

  - UART mode: GNSS UART and compass I2C stay on the normally-closed
    switch path straight to the host connector. The MCU only listens to
    GNSS_TX through a 10k tap and drives the status LED.
  - CAN mode: the switches route GNSS and compass to the MCU and the
    module acts as a normal DroneCAN GPS/compass node.
 */
#pragma once

#ifdef HAL_CORVON_GPS_ENABLED

#include <stdint.h>

class CorvonGPS {
public:
    enum class Mode : uint8_t {
        BOOT = 0,
        UART_DIRECT,
        CAN,
    };

    // sense connector power, latch the mode and set the switches.
    // must run after load_parameters() and before gps.init()
    void init(void);

    // 50Hz: run the status LED pattern engine
    void update(void);

    Mode get_mode(void) const { return mode; }
    bool direct_mode(void) const { return mode == Mode::UART_DIRECT; }

    // called when the flight controller sends a LightsCommand so the
    // local patterns yield to it
    void note_lights_command(void);

private:
    Mode mode = Mode::BOOT;
    bool dual_power;
    uint32_t boot_ms;
    uint32_t last_lights_ms;
    uint32_t last_slot_ms;
    uint8_t slot;

    void set_led(uint8_t r, uint8_t g, uint8_t b);
};

#endif // HAL_CORVON_GPS_ENABLED
