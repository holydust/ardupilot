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

// engineering tool: a console command that can force BOR_LEV. Off in shipped
// firmware - see the comment on cmd_bor() in corvon_gps.cpp for why
#ifndef CORVON_BOR_TOOL_ENABLED
#define CORVON_BOR_TOOL_ENABLED 0
#endif

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

    // called from the CAN rx path. A node that hears nothing at all is
    // either unplugged or on a bus with no other node, which is worth
    // showing on the LED
    void note_can_rx(void);

private:
    Mode mode = Mode::BOOT;
    bool dual_power;
    // millis() at init(), and again at the first update(). The LED
    // patterns hang off the second one: everything between the two is
    // gps/compass/baro/imu init, during which nothing can reach the LED
    uint32_t init_ms;
    uint32_t boot_ms;
    uint32_t last_lights_ms;
    uint32_t last_can_rx_ms;

    // production self-test, triggered by typing "test" on the debug
    // console. test_end_ms drives the LED inspection cycle
    uint32_t test_end_ms;
    uint32_t last_pps_ms;
    bool pps_last_state;
    // long enough for "set COMPASS_ORIENT -1" plus the terminator
    char cmd_buf[48];
    uint8_t cmd_len;
    // set when a line overflows cmd_buf, cleared at the next line
    // terminator. Everything up to that terminator is thrown away
    bool discarding;

    // one bit per whitelisted parameter changed since boot, so "save"
    // writes only what the operator actually touched
    uint32_t dirty_mask;

    void check_console(void);
    void run_command(char *line);
    void run_selftest(void);
    void cmd_version(void);
    void cmd_mag(void);
    void cmd_list(void);
    void cmd_get(const char *name);
    void cmd_set(const char *name, const char *value);
    void cmd_save(void);

    // BOR Level 3 set from the application, see corvon_gps.cpp
    void bor_init(void);
    const char *bor_write_level(uint8_t lev);
    const char *bor_status = nullptr;        // nullptr means look at bor_fail_reason
    const char *bor_fail_reason = nullptr;
#if CORVON_BOR_TOOL_ENABLED
    void cmd_bor(const char *arg);        // engineering tool, see corvon_gps.cpp
#endif
#if CORVON_POOL_DIAG_ENABLED
    void cmd_pool(void);                  // engineering tool, see corvon_gps.cpp
#endif

    // one-shot power-on sequence: colour sweep then the mode colour
    void boot_pattern(uint32_t t);
    // warning mark in the tail of each cycle, highest priority only.
    // returns true if it drew something
    bool draw_warning(uint32_t t);
    // n short flashes at the start of a 2s cycle
    void flash_n(uint32_t cyc, uint8_t n, uint8_t r, uint8_t g, uint8_t b);

    void set_led(uint8_t r, uint8_t g, uint8_t b);
    // same, scaled by level/255 on top of NTF_LED_BRIGHT
    void set_led_dim(uint8_t r, uint8_t g, uint8_t b, uint8_t level);
};

#endif // HAL_CORVON_GPS_ENABLED
