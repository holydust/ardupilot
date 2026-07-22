#include "AP_Periph.h"

#ifdef HAL_CORVON_GPS_ENABLED

#include <AP_Param/AP_Param.h>
#include <string.h>

extern const AP_HAL::HAL &hal;

// GPIO() numbers assigned in the hwdef
#define CORVON_GPIO_SW_IN1   60  // RS2058 selects, high = MCU path
#define CORVON_GPIO_SW_IN2   61
#define CORVON_GPIO_DET_UART 62  // per-connector 5V sense, pre diode-OR
#define CORVON_GPIO_DET_CAN  63
#define CORVON_GPIO_PPS      66  // GNSS TIMEPULSE

void CorvonGPS::init(void)
{
    boot_ms = AP_HAL::millis();

    // sample both presence lines for 50ms. The 10K/10K dividers give a
    // clean logic level; sampling only guards against power-on bounce
    uint8_t uart_hits = 0, can_hits = 0;
    for (uint8_t i = 0; i < 10; i++) {
        uart_hits += hal.gpio->read(CORVON_GPIO_DET_UART) ? 1 : 0;
        can_hits += hal.gpio->read(CORVON_GPIO_DET_CAN) ? 1 : 0;
        hal.scheduler->delay(5);
    }
    const bool uart_present = uart_hits >= 7;
    const bool can_present = can_hits >= 7;

    // both connectors powered: CAN wins, warn on the LED
    dual_power = uart_present && can_present;

    if (can_present) {
        // CAN mode: route GNSS UART and compass I2C to the MCU. The
        // I2C driver recovers the bus if a transaction was cut short
        mode = Mode::CAN;
        hal.gpio->write(CORVON_GPIO_SW_IN1, 1);
        hal.gpio->write(CORVON_GPIO_SW_IN2, 1);
        return;
    }

    // UART mode, also the fallback when nothing is sensed (bench
    // power): switches stay in NC = passthrough, we listen through the
    // 10k tap. The receiver belongs to the flight controller, so never
    // configure it; USART1 TX is physically disconnected by the open
    // switch leg but keep auto-config off as well
    mode = Mode::UART_DIRECT;
    AP_Param::set_by_name("GPS_AUTO_CONFIG", 0);
}

void CorvonGPS::note_lights_command(void)
{
    last_lights_ms = AP_HAL::millis();
}

void CorvonGPS::set_led(uint8_t r, uint8_t g, uint8_t b)
{
#if AP_PERIPH_NOTIFY_ENABLED
    periph.notify.handle_rgb(r, g, b);
#endif
}

// poll the debug console for the production test command
void CorvonGPS::check_console(void)
{
    auto *uart = hal.serial(0);
    if (uart == nullptr || !uart->is_initialized()) {
        return;
    }
    uint32_t n = MIN(uart->available(), 64U);
    while (n-- > 0) {
        uint8_t c;
        if (!uart->read(c)) {
            break;
        }
        if (c == '\r' || c == '\n') {
            cmd_buf[cmd_len] = 0;
            cmd_len = 0;
            if (strcmp(cmd_buf, "test") == 0) {
                run_selftest();
            }
        } else if (cmd_len < sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_len++] = char(c);
        }
    }
}

/*
  production self-test: print a PASS/FAIL report on the debug console
  and cycle the LED red-green-blue-white for visual inspection.

  the test jig must power the CAN connector: in UART mode the compass
  I2C bus is switched to the host and cannot be checked from here
 */
void CorvonGPS::run_selftest(void)
{
    auto &uart = *hal.serial(0);
    const uint32_t now = AP_HAL::millis();

    uart.printf("\nCORVON selftest\n");

    char sysid[50];
    if (hal.util->get_system_id(sysid)) {
        uart.printf("id: %s\n", sysid);
    }

    uart.printf("mode: %s%s\n",
                mode == Mode::CAN ? "CAN" : "UART-direct",
                dual_power ? " (both connectors powered!)" : "");
    uart.printf("det: uart=%u can=%u\n",
                unsigned(hal.gpio->read(CORVON_GPIO_DET_UART)),
                unsigned(hal.gpio->read(CORVON_GPIO_DET_CAN)));

    // a recently parsed message proves the UART path and the receiver
    const uint32_t last_msg = periph.gps.last_message_time_ms();
    const bool gnss_ok = last_msg != 0 && now - last_msg < 2000;
    uart.printf("gnss: %s fix=%u sats=%u\n", gnss_ok ? "PASS" : "FAIL",
                unsigned(periph.gps.status()), unsigned(periph.gps.num_sats()));

    // TIMEPULSE runs at 1Hz before any fix, so this only proves the
    // GNSS core is alive and the PPS trace is intact
    const bool pps_ok = last_pps_ms != 0 && now - last_pps_ms < 2500;
    uart.printf("pps: %s\n", pps_ok ? "PASS" : "FAIL");

    bool mag_ok = true;
#if AP_PERIPH_MAG_ENABLED
    if (mode == Mode::CAN) {
        // field magnitude in a plausible Earth-field range proves the
        // sensor returns real data, not just an ID
        const float field = periph.compass.get_field().length();
        mag_ok = periph.compass.get_count() > 0 && periph.compass.healthy()
                 && field > 100 && field < 1000;
        uart.printf("mag: %s n=%u field=%.0f mGa\n", mag_ok ? "PASS" : "FAIL",
                    unsigned(periph.compass.get_count()), field);
    } else {
        uart.printf("mag: SKIP (host owns the I2C bus in UART mode)\n");
    }
#endif

    test_end_ms = now + 4000;
    uart.printf("led: red-green-blue-white for 4s, check by eye\n");

    uart.printf("result: %s\n", (gnss_ok && pps_ok && mag_ok) ? "PASS" : "FAIL");
}

/*
  status LED patterns, 16 slots of 125ms = 2s cycle:

  boot (first 1.5s)        white fast blink
  no data seen (UART mode) white slow blink   (baudrate not found yet)
  no fix                   blue blink
  2D fix                   green blink
  3D fix or better         green solid, RTK float/fixed add a short
                           blue/white mark each cycle
  both connectors powered  orange mark at end of each cycle
 */
void CorvonGPS::update(void)
{
    const uint32_t now = AP_HAL::millis();

    check_console();

    // PPS edge tracking for the self-test. The 100ms TIMEPULSE high
    // phase is much longer than our 20ms poll interval
    const bool pps = hal.gpio->read(CORVON_GPIO_PPS);
    if (pps && !pps_last_state) {
        last_pps_ms = now;
    }
    pps_last_state = pps;

    // self-test LED cycle overrides all patterns including the FC yield
    if (test_end_ms != 0) {
        if (now < test_end_ms) {
            switch ((now / 500) % 4) {
            case 0: set_led(255, 0, 0);     break;
            case 1: set_led(0, 255, 0);     break;
            case 2: set_led(0, 0, 255);     break;
            case 3: set_led(255, 255, 255); break;
            }
            return;
        }
        test_end_ms = 0;
    }

    // in CAN mode the flight controller owns the LED while it is
    // actively sending LightsCommand
    if (mode == Mode::CAN && last_lights_ms != 0 && now - last_lights_ms < 3000) {
        return;
    }

    if (now - last_slot_ms < 125) {
        return;
    }
    last_slot_ms = now;
    slot = (slot + 1) % 16;

    const uint8_t B = 90;  // brightness cap

    if (now - boot_ms < 1500) {
        const uint8_t v = (slot & 1) ? B : 0;
        set_led(v, v, v);
        return;
    }

    // dual-power warning overrides the last two slots of each cycle
    if (dual_power && slot >= 14) {
        set_led(B, B/2, 0);
        return;
    }

    const AP_GPS::GPS_Status status = periph.gps.status();

    if (mode == Mode::UART_DIRECT && periph.gps.last_message_time_ms() == 0) {
        // listening but nothing parsed yet: slow white
        set_led(slot < 8 ? B : 0, slot < 8 ? B : 0, slot < 8 ? B : 0);
        return;
    }

    switch (status) {
    case AP_GPS::NO_GPS:
    case AP_GPS::NO_FIX:
        // searching: blue 1Hz
        set_led(0, 0, (slot % 8) < 4 ? B : 0);
        break;
    case AP_GPS::GPS_OK_FIX_2D:
        // 2D: green 1Hz
        set_led(0, (slot % 8) < 4 ? B : 0, 0);
        break;
    case AP_GPS::GPS_OK_FIX_3D:
    case AP_GPS::GPS_OK_FIX_3D_DGPS:
    case AP_GPS::GPS_OK_FIX_TYPE_STATIC:
    case AP_GPS::GPS_OK_FIX_TYPE_PPP:
        // 3D: green solid
        set_led(0, B, 0);
        break;
    case AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT:
        // RTK float: green with a blue mark
        if (slot < 2) {
            set_led(0, 0, B);
        } else {
            set_led(0, B, 0);
        }
        break;
    case AP_GPS::GPS_OK_FIX_3D_RTK_FIXED:
        // RTK fixed: green with a white mark
        if (slot < 2) {
            set_led(B, B, B);
        } else {
            set_led(0, B, 0);
        }
        break;
    }
}

#endif // HAL_CORVON_GPS_ENABLED
