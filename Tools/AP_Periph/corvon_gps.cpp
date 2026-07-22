#include "AP_Periph.h"

#ifdef HAL_CORVON_GPS_ENABLED

#include <AP_Param/AP_Param.h>

extern const AP_HAL::HAL &hal;

// GPIO() numbers assigned in the hwdef
#define CORVON_GPIO_SW_IN1   60  // RS2058 selects, high = MCU path
#define CORVON_GPIO_SW_IN2   61
#define CORVON_GPIO_DET_UART 62  // per-connector 5V sense, pre diode-OR
#define CORVON_GPIO_DET_CAN  63

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
