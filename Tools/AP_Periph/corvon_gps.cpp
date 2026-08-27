#include "AP_Periph.h"

#ifdef HAL_CORVON_GPS_ENABLED

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_FWVersion.h>
#include <string.h>
#include <stdlib.h>

extern const AP_HAL::HAL &hal;

// GPIO() numbers assigned in the hwdef
#define CORVON_GPIO_SW_IN1   60  // RS2058 selects, high = MCU path
#define CORVON_GPIO_SW_IN2   61
#define CORVON_GPIO_DET_UART 62  // per-connector 5V sense, pre diode-OR
#define CORVON_GPIO_DET_CAN  63
#define CORVON_GPIO_PPS      66  // GNSS TIMEPULSE
#define CORVON_SELFTEST_MS 4000  // "test" LED inspection cycle

void CorvonGPS::init(void)
{
    init_ms = AP_HAL::millis();

    // before anything here can reach flash. A board whose level is already
    // right takes no write at all, which is every board after the line
    bor_init();
    boot_ms = 0;                  // anchored at the first update(), see there

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

void CorvonGPS::note_can_rx(void)
{
    last_can_rx_ms = AP_HAL::millis();
}

/*
  NTF_LED_BRIGHT has to be applied here. RGBLed::rgb_control() stores
  the override verbatim and nothing downstream scales an AP_Periph rgb
  override, so a brightness parameter has no effect unless we do it
 */
void CorvonGPS::set_led(uint8_t r, uint8_t g, uint8_t b)
{
#if AP_PERIPH_NOTIFY_ENABLED
    int8_t pct = periph.notify.get_rgb_led_brightness_percent();
    if (pct < 0 || pct > 100) {
        pct = 100;
    }
    r = (uint16_t(r) * pct) / 100;
    g = (uint16_t(g) * pct) / 100;
    b = (uint16_t(b) * pct) / 100;
    periph.notify.handle_rgb(r, g, b);
#endif
}

void CorvonGPS::set_led_dim(uint8_t r, uint8_t g, uint8_t b, uint8_t level)
{
    set_led((uint16_t(r) * level) / 255,
            (uint16_t(g) * level) / 255,
            (uint16_t(b) * level) / 255);
}

/*
  parameters the configuration tool is allowed to touch. Deliberately a
  short list rather than the whole AP_Param tree: everything here is
  something a customer may reasonably change, and nothing here can brick
  the node beyond a re-flash
 */
static const struct {
    const char *name;
    float min;
    float max;
    bool reboot_required;
    const char *help;
} corvon_params[] = {
    { "CAN_NODE",        0,     125,     true,  "DroneCAN node id, 0=auto (DNA)" },
    { "CAN_BAUDRATE",    10000, 1000000, true,  "CAN bitrate" },
    // not COMPASS_ORIENT or COMPASS_USE: AP_Compass keeps both behind
    // #ifndef HAL_BUILD_AP_PERIPH, so neither exists in a periph build and
    // every get/set of them answered "err no such param". Rotation is a
    // build-time property here - it comes from the ROTATION_ROLL_180 on the
    // COMPASS line in the board's hwdef.dat, and changing it means a new
    // firmware, not a parameter write.
    // not LED_BRIGHTNESS: that one lives behind
    // AP_PERIPH_HAVE_LED_WITHOUT_NOTIFY and is not compiled in on a
    // board that drives its LED through AP_Notify
    { "NTF_LED_BRIGHT",  0,     3,       false, "status LED brightness 0-3" },
    { "GPS1_RATE_MS",    50,    1000,    false, "GNSS output period in ms" },
};

// poll the debug console for commands
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
            if (cmd_len > 0 && !discarding) {
                run_command(cmd_buf);
            }
            cmd_len = 0;
            discarding = false;
        } else if (discarding) {
            // still inside a line that already overflowed
        } else if (cmd_len < sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_len++] = char(c);
        } else {
            // overlong line: drop the whole line, not just the head. Line
            // noise must never leave a tail behind that parses as a command
            cmd_len = 0;
            discarding = true;
        }
    }
}

// split a completed line into verb and up to two arguments
void CorvonGPS::run_command(char *line)
{
    char *saveptr = nullptr;
    const char *verb = strtok_r(line, " \t", &saveptr);
    if (verb == nullptr) {
        return;
    }
    const char *arg1 = strtok_r(nullptr, " \t", &saveptr);
    const char *arg2 = strtok_r(nullptr, " \t", &saveptr);

    if (strcmp(verb, "test") == 0) {
        run_selftest();
    } else if (strcmp(verb, "ver") == 0) {
        cmd_version();
    } else if (strcmp(verb, "mag") == 0) {
        cmd_mag();
    } else if (strcmp(verb, "list") == 0) {
        cmd_list();
    } else if (strcmp(verb, "get") == 0) {
        cmd_get(arg1);
    } else if (strcmp(verb, "set") == 0) {
        cmd_set(arg1, arg2);
    } else if (strcmp(verb, "save") == 0) {
        cmd_save();
#if CORVON_BOR_TOOL_ENABLED
    } else if (strcmp(verb, "bor") == 0) {
        cmd_bor(arg1);
#endif
#if CORVON_POOL_DIAG_ENABLED
    } else if (strcmp(verb, "pool") == 0) {
        cmd_pool();
#endif
    } else if (strcmp(verb, "reboot") == 0) {
        // Plain "reboot" is what you want after changing CAN_NODE /
        // CAN_BAUDRATE. "reboot -b" comes back up in the bootloader.
        //
        // The -b spelling is not decoration: it is the string uploader.py
        // sends to ask a running board to make itself available for an
        // upload. AP_Periph already matches that sequence in
        // check_for_serial_reboot_cmd() and answers it with reboot(true) -
        // but this console reads the same UART and consumes the bytes first,
        // so the matcher never sees a contiguous match. Without this the
        // request quietly became a reboot into the application, the upload
        // could only catch the bootloader's 1s window by luck, and the
        // documented workaround was to power cycle the board by hand.
        const bool hold_in_bl = arg1 != nullptr && strcmp(arg1, "-b") == 0;
        hal.serial(0)->printf("ok rebooting%s\n", hold_in_bl ? " to bootloader" : "");
        hal.scheduler->delay(50);   // let the reply drain
        periph.prepare_reboot();
        hal.scheduler->reboot(hold_in_bl);
    } else {
        hal.serial(0)->printf("err unknown command\n");
    }
}

/*
  protocol version for the configuration tool. Bump CORVON_PROTO_VERSION
  whenever a command's syntax or output changes so the tool can adapt
 */
// 2: selftest "result:" gained INCOMPLETE alongside PASS and FAIL
// 3: "reboot -b" reboots into the bootloader (older firmware ignored the
//    argument and rebooted into the application), reply text changed
#define CORVON_PROTO_VERSION 3

// raw field vector, for confirming COMPASS_ORIENT at bring-up. The
// self-test only reports the magnitude, which is identical whichever way
// the part is turned, so it cannot catch a wrong rotation
#if CORVON_POOL_DIAG_ENABLED
/*
  Engineering tool. Reports canard pool occupancy and the transmit failure
  counters, to tell apart three things that all look like "ENLARGE MEMORY
  POOL" on the console:

    tx_fail rising     - frames are not reaching the bus at all, so the queue
                         cannot drain and the pool fills behind it. A CAN
                         transceiver / bus / termination fault, not a size
                         problem.
    tx_oom rising      - the pool genuinely ran out and whole transfers were
                         refused before a single frame was queued. libcanard
                         does that deliberately rather than emit a corrupt
                         partial transfer, so the loss is silent.
    cur low, peak 100% - a spike between two 1Hz cleanups. Nothing was lost;
                         the warning is driven by a high water mark that never
                         decays.

  Not compiled into shipped firmware: the numbers are only meaningful next to
  a known bus, and a console command that prints internal counters invites
  them being quoted as a health check.
 */
void CorvonGPS::cmd_pool(void)
{
    auto &uart = *hal.serial(0);
    AP_Periph_FW::CorvonPoolDiag d {};
    periph.corvon_pool_diag(d);
    const uint32_t pct = d.cap_blocks ? (100U * d.peak_blocks) / d.cap_blocks : 0;
    const uint32_t cur_pct = d.cap_blocks ? (100U * d.cur_blocks) / d.cap_blocks : 0;
    uart.printf("pool: cur %u/%u (%u%%) peak %u (%u%%) tx_oom %lu tx_err %u "
                "tx_streak %u rx_oom %u tx_frames %lu rx_ok %lu rx_ign %lu rx_bad %lu\n",
                unsigned(d.cur_blocks), unsigned(d.cap_blocks), unsigned(cur_pct),
                unsigned(d.peak_blocks), unsigned(pct),
                (unsigned long)d.tx_oom, unsigned(d.tx_errors),
                unsigned(d.tx_fail_count), unsigned(d.rx_error_oom),
                (unsigned long)d.tx_frames, (unsigned long)d.rx_frames,
                (unsigned long)d.rx_ignored, (unsigned long)d.rx_bad);
    uart.printf("ok\n");
}
#endif // CORVON_POOL_DIAG_ENABLED

void CorvonGPS::cmd_mag(void)
{
    auto &uart = *hal.serial(0);
#if AP_PERIPH_MAG_ENABLED
    if (periph.compass.get_count() == 0) {
        // every command terminates with ok or err - a bare line leaves a
        // host waiting for a terminator that never comes, and reports a
        // board that answered correctly as an unresponsive console
        uart.printf("mag: none\nok\n");
        return;
    }
    const Vector3f f = periph.compass.get_field();
    uart.printf("mag: x=%+.0f y=%+.0f z=%+.0f mGa |B|=%.0f healthy=%u\n",
                f.x, f.y, f.z, f.length(), unsigned(periph.compass.healthy()));
    // level and pointing north in the northern hemisphere: x is the
    // largest positive term, y sits near zero, z is positive because the
    // field dips downwards. y flipping sign means the rotation is 180 out
    uart.printf("ok\n");
#else
    uart.printf("mag: not compiled in\nok\n");
#endif
}

/*
  BOR Level 3 without a debugger.

  The option bytes were previously only reachable over SWD, which made a board
  with a dead debug port unshippable however healthy the rest of it was. Upstream
  programs option bytes from the running application on H7 and the G/L parts, but
  those helpers are compiled out on F4, so this is our own sequence - the same one
  Tools/corvon/bor/bor_program.cfg drives through openocd.

  Proven on a CORVON-G1 (APM32F405) on 2026-08-26: written from the application in
  both directions and read back unchanged after the rail was measured down to
  0.01V, so it really commits to NVM rather than sitting in a controller buffer.
  bor_verify.cfg then reported GATE2 PASS on the restored value.

  Two properties worth keeping in mind before touching this:

  - It only writes when the level is already wrong, so a board takes one option
    byte write in its life and this path is dormant in the field forever after.
  - Nothing but BOR_LEV is ever modified. Writing the RDP field by accident
    triggers a mass erase, which is the one way this code could destroy a board,
    so it refuses to start unless RDP already reads back as 0xAA.
*/
#define CORVON_FLASH_SR      (*(volatile uint32_t *)0x40023C0C)
#define CORVON_FLASH_OPTKEYR (*(volatile uint32_t *)0x40023C08)
#define CORVON_FLASH_OPTCR   (*(volatile uint32_t *)0x40023C14)
#define CORVON_OPT_STORAGE   (*(volatile uint32_t *)0x1FFFC000)
#define CORVON_FLASH_BSY     (1U << 16)
#define CORVON_OPTCR_OPTLOCK (1U << 0)
#define CORVON_OPTCR_OPTSTRT (1U << 1)
#define CORVON_BOR_LEVEL3    0    // BOR_LEV encoding: 0 = ~2.7V, 3 = off

/*
  returns nullptr on success, else a short reason suitable for the ver line
*/
const char *CorvonGPS::bor_write_level(uint8_t lev)
{
    if ((CORVON_FLASH_OPTCR >> 8 & 0xFF) != 0xAA) {
        return "rdp-not-0xAA";
    }
    if (CORVON_FLASH_SR & CORVON_FLASH_BSY) {
        return "flash-busy";
    }
    if (CORVON_FLASH_OPTCR & CORVON_OPTCR_OPTLOCK) {
        CORVON_FLASH_OPTKEYR = 0x08192A3B;
        CORVON_FLASH_OPTKEYR = 0x4C5D6E7F;
    }
    if (CORVON_FLASH_OPTCR & CORVON_OPTCR_OPTLOCK) {
        return "unlock-failed";
    }

    // clear BOR_LEV/OPTSTRT/OPTLOCK only, so RDP and nWRP carry through
    uint32_t v = CORVON_FLASH_OPTCR;
    v &= ~0xFU;
    v |= (uint32_t(lev) & 3) << 2;
    CORVON_FLASH_OPTCR = v;
    CORVON_FLASH_OPTCR = v | CORVON_OPTCR_OPTSTRT;

    for (uint32_t i = 0; i < 1000000 && (CORVON_FLASH_SR & CORVON_FLASH_BSY); i++) { }
    CORVON_FLASH_OPTCR |= CORVON_OPTCR_OPTLOCK;

    /*
      BSY is not a completion signal on this silicon - both measured writes were
      already finished before the first poll, so the loop above bounds the wait
      and proves nothing. Worse, 0x1FFFC000 passes through a state whose
      complement does not check out while programming settles: a read taken
      straight after OPTSTRT returned 0x5500AAF3, value and complement
      disagreeing, and the same address read 0x5500AAFF a second later
      (measured on a CORVON-G1, 2026-08-26).

      So completion is decided by reading it back until it is self-consistent
      AND carries the level asked for. 20 attempts at 5ms is 100ms of headroom
      over a write that has so far always been instant.
    */
    const char *last = "storage-unchanged";
    for (uint8_t attempt = 0; attempt < 20; attempt++) {
        hal.scheduler->delay(5);
        if (((CORVON_FLASH_OPTCR >> 2) & 3) != (uint32_t(lev) & 3)) {
            last = "optcr-unchanged";
            continue;
        }
        const uint32_t st = CORVON_OPT_STORAGE;
        const uint16_t val = st & 0xFFFF;
        const uint16_t cpl = (st >> 16) & 0xFFFF;
        if (cpl != uint16_t(~val)) {
            last = "storage-torn";
            continue;
        }
        if (((val >> 2) & 3) != (uint32_t(lev) & 3)) {
            last = "storage-unchanged";
            continue;
        }
        return nullptr;
    }
    return last;
}

/*
  called from init(), before anything on this board can write flash
*/
void CorvonGPS::bor_init(void)
{
    const uint8_t lev = (CORVON_FLASH_OPTCR >> 2) & 3;
    if (lev == CORVON_BOR_LEVEL3) {
        bor_status = "ok level3";
        return;
    }
    const char *err = bor_write_level(CORVON_BOR_LEVEL3);
    if (err != nullptr) {
        bor_fail_reason = err;
        bor_status = nullptr;      // ver prints the reason instead
        return;
    }
    bor_status = "ok level3 (set this boot)";
}


/*
  Engineering tool, off in shipped firmware. Build with
  -DCORVON_BOR_TOOL_ENABLED=1 to get a "bor [0-3]" console command that reads
  the option bytes and can force an arbitrary BOR_LEV. It exists because the
  only way to test the production path above is to put a board back into the
  state a virgin one arrives in, and because it is the fastest way to read the
  option bytes on a board whose SWD pads are unusable.

  Not shipped: a console command that writes option bytes is a standing risk
  surface for no production benefit, since bor_init() already does the one
  write a board ever needs.
*/
#if CORVON_BOR_TOOL_ENABLED
void CorvonGPS::cmd_bor(const char *arg)
{
    auto &uart = *hal.serial(0);
    uart.printf("optcr:   0x%08x  bor_lev %u  rdp 0x%02x\n",
                unsigned(CORVON_FLASH_OPTCR),
                unsigned((CORVON_FLASH_OPTCR >> 2) & 3),
                unsigned((CORVON_FLASH_OPTCR >> 8) & 0xFF));
    uart.printf("storage: 0x%08x  (0x1FFFC000)\n", unsigned(CORVON_OPT_STORAGE));
    if (arg == nullptr) {
        uart.printf("ok\n");
        return;
    }
    char *end = nullptr;
    const long want = strtol(arg, &end, 0);
    if (end == arg || (end != nullptr && *end != 0) || want < 0 || want > 3) {
        uart.printf("err usage: bor [0-3]   raw BOR_LEV, 0=Level3(2.7V) 3=off\n");
        return;
    }
    const char *err = bor_write_level(uint8_t(want));
    if (err != nullptr) {
        uart.printf("err %s\n", err);
        return;
    }
    uart.printf("optcr:   0x%08x  bor_lev %u\n",
                unsigned(CORVON_FLASH_OPTCR), unsigned((CORVON_FLASH_OPTCR >> 2) & 3));
    uart.printf("storage: 0x%08x\nok\n", unsigned(CORVON_OPT_STORAGE));
}
#endif // CORVON_BOR_TOOL_ENABLED

void CorvonGPS::cmd_version(void)
{
    auto &uart = *hal.serial(0);
    uart.printf("proto: %u\n", (unsigned)CORVON_PROTO_VERSION);
    uart.printf("fw: %s %s\n", CHIBIOS_BOARD_NAME, AP::fwversion().fw_string);
#ifdef APJ_BOARD_ID
    uart.printf("board_id: %u\n", (unsigned)APJ_BOARD_ID);
#endif
    uart.printf("mode: %s\n", mode == Mode::CAN ? "CAN" : "UART-direct");
    // gap between init() and the first update(). The LED cannot be driven
    // during it, so the boot sweep is anchored past it rather than at
    // init(): on a DW609 this measured long enough to eat the red step
    uart.printf("led_anchor_delay: %ums\n", unsigned(boot_ms - init_ms));
    // production gate reads this line: pass is exactly the "bor: ok" prefix
    if (bor_status != nullptr) {
        uart.printf("bor: %s\n", bor_status);
    } else {
        uart.printf("bor: FAIL %s lev %u rdp 0x%02x\n",
                    bor_fail_reason != nullptr ? bor_fail_reason : "unknown",
                    unsigned((CORVON_FLASH_OPTCR >> 2) & 3),
                    unsigned((CORVON_FLASH_OPTCR >> 8) & 0xFF));
    }
    uart.printf("ok\n");
}

void CorvonGPS::cmd_list(void)
{
    auto &uart = *hal.serial(0);
    for (const auto &p : corvon_params) {
        float v = 0;
        if (!AP_Param::get(p.name, v)) {
            // a name in the table that the firmware does not actually
            // have. Report it rather than hiding the row, otherwise a
            // typo here looks like a missing feature at the far end
            uart.printf("%s MISSING\n", p.name);
            continue;
        }
        uart.printf("%s %.4g %.4g %.4g %s\n", p.name, (double)v,
                    (double)p.min, (double)p.max, p.help);
    }
    uart.printf("ok\n");
}

// look up a name in the whitelist, nullptr if it is not settable
static uint8_t corvon_param_index(const char *name)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(corvon_params); i++) {
        if (strcmp(corvon_params[i].name, name) == 0) {
            return i;
        }
    }
    return UINT8_MAX;
}

void CorvonGPS::cmd_get(const char *name)
{
    auto &uart = *hal.serial(0);
    if (name == nullptr) {
        uart.printf("err usage: get <name>\n");
        return;
    }
    if (corvon_param_index(name) == UINT8_MAX) {
        uart.printf("err not exposed\n");
        return;
    }
    float v = 0;
    if (!AP_Param::get(name, v)) {
        uart.printf("err no such param\n");
        return;
    }
    uart.printf("%s %.4g\n", name, (double)v);
    uart.printf("ok\n");
}

void CorvonGPS::cmd_set(const char *name, const char *value)
{
    auto &uart = *hal.serial(0);
    if (name == nullptr || value == nullptr) {
        uart.printf("err usage: set <name> <value>\n");
        return;
    }
    const uint8_t idx = corvon_param_index(name);
    if (idx == UINT8_MAX) {
        uart.printf("err not exposed\n");
        return;
    }
    char *end = nullptr;
    const float v = strtof(value, &end);
    if (end == value || (end != nullptr && *end != 0)) {
        uart.printf("err not a number\n");
        return;
    }
    if (v < corvon_params[idx].min || v > corvon_params[idx].max) {
        uart.printf("err out of range %.4g..%.4g\n",
                    (double)corvon_params[idx].min, (double)corvon_params[idx].max);
        return;
    }
    if (!AP_Param::set_by_name(name, v)) {
        uart.printf("err set failed\n");
        return;
    }
    // the value is live but volatile until "save"
    dirty_mask |= 1U << idx;
    uart.printf("ok\n");
}

/*
  write the parameters touched since boot back to storage. set_by_name()
  only updated RAM, so re-read each live value and save that. Untouched
  parameters are left alone so "save" never persists a default the
  operator did not ask for
 */
void CorvonGPS::cmd_save(void)
{
    auto &uart = *hal.serial(0);
    if (dirty_mask == 0) {
        uart.printf("nothing to save\nok\n");
        return;
    }
    bool needs_reboot = false;
    for (uint8_t i = 0; i < ARRAY_SIZE(corvon_params); i++) {
        if ((dirty_mask & (1U << i)) == 0) {
            continue;
        }
        float v = 0;
        // not the _ifchanged variant: "set" already wrote the live value, so
        // ifchanged sees nothing to do and returns success without touching
        // storage. AP_Param.h says as much - it is only safe where set() was
        // not called separately.
        if (!AP_Param::get(corvon_params[i].name, v) ||
            !AP_Param::set_and_save_by_name(corvon_params[i].name, v)) {
            uart.printf("err save failed at %s\n", corvon_params[i].name);
            return;
        }
        uart.printf("saved %s %.4g\n", corvon_params[i].name, (double)v);
        needs_reboot |= corvon_params[i].reboot_required;
    }
    dirty_mask = 0;
    uart.printf(needs_reboot ? "ok reboot required\n" : "ok\n");
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

    // TIMEPULSE only appears once the receiver has time lock. This build
    // leaves CONFIGURE_PPS_PIN at 0, so ArduPilot never touches CFG-TP5 and
    // the u-blox default applies: zero pulse length while unlocked, 100ms
    // once locked. So this check needs a fix to pass, and a production
    // fixture needs an antenna and usable signal or every board reads FAIL.
    // Measured on the first DW608 on 2026-08-19: indoors, no fix, PA0 flat.
    const bool pps_ok = last_pps_ms != 0 && now - last_pps_ms < 2500;
    uart.printf("pps: %s\n", pps_ok ? "PASS" : "FAIL");

    bool mag_ok = true;
    // a build with no compass at all has nothing to cover. Only the
    // UART path leaves a compass that is fitted but unreachable
    bool mag_covered = true;
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
        mag_covered = false;
        uart.printf("mag: SKIP (host owns the I2C bus in UART mode)\n");
    }
#endif

    test_end_ms = now + CORVON_SELFTEST_MS;
    uart.printf("led: red-green-blue-white for 4s, check by eye\n");

    // a run that could not reach the compass must not read as PASS. A
    // production fixture greps this line, and powering the board from
    // the wrong connector would otherwise ship untested compasses
    if (!gnss_ok || !pps_ok || !mag_ok) {
        uart.printf("result: FAIL\n");
    } else if (!mag_covered) {
        uart.printf("result: INCOMPLETE mag not covered, "
                    "power the CAN connector for a full test\n");
    } else {
        uart.printf("result: PASS\n");
    }
}

/*
  status LED. Two layers:

  - a one-shot power-on sequence: a red-green-blue sweep, which proves
    all three channels of the single WS2812, followed by two pulses of
    the latched interface mode (cyan = UART, magenta = CAN)
  - a repeating 2s cycle carrying the GNSS state, with at most one
    warning mark in the last 500ms

  Breathing is used for every healthy ongoing state and hard flashes
  for anything wanting attention, so the states stay apart even when
  sunlight washes the colour out. The breath is floored well above
  zero: a pattern that reaches full dark reads as "LED dead" outdoors.

  In CAN mode a connected flight controller takes the LED over with
  LightsCommand, so these patterns are what you see in UART mode and
  in CAN mode before the flight controller is up - which is exactly
  when the diagnosis matters.

  Two states need no code and are documented for support only: the
  power-on sweep repeating forever means the board is boot looping
  (nearly always a sagging supply), and a dark or frozen LED means no
  power, no firmware, or a dead LED.
 */

#define CORVON_BOOT_SWEEP_MS   900    // 3 x 300ms colour sweep
#define CORVON_BOOT_SEQ_MS    1900    // sweep + two mode pulses
#define CORVON_GNSS_GRACE_MS  5000    // silence below this is just booting
#define CORVON_CYCLE_MS       2000
#define CORVON_WARN_MS         500    // warning mark occupies the tail

// triangle breath across the cycle, floored at 25% of the set level
static uint8_t breathe_level(uint32_t cyc)
{
    const uint32_t half = CORVON_CYCLE_MS / 2;
    const uint32_t up = (cyc < half) ? cyc : (CORVON_CYCLE_MS - cyc);
    return 64 + (up * 191) / half;
}

void CorvonGPS::flash_n(uint32_t cyc, uint8_t n, uint8_t r, uint8_t g, uint8_t b)
{
    const uint32_t period = 200;   // 120ms on, 80ms off
    if (cyc < n * period && (cyc % period) < 120) {
        set_led(r, g, b);
    } else {
        set_led(0, 0, 0);
    }
}

void CorvonGPS::boot_pattern(uint32_t t)
{
    if (t < 300) {
        set_led(255, 0, 0);
    } else if (t < 600) {
        set_led(0, 255, 0);
    } else if (t < CORVON_BOOT_SWEEP_MS) {
        set_led(0, 0, 255);
    } else {
        // two pulses of the latched mode colour. These two colours are
        // used nowhere else, so they cannot be read as a fix state
        const bool on = ((t - CORVON_BOOT_SWEEP_MS) % 500) < 300;
        const uint8_t v = on ? 255 : 0;
        if (mode == Mode::CAN) {
            set_led(v, 0, v);          // magenta
        } else {
            set_led(0, v, v);          // cyan
        }
    }
}

/*
  one warning mark per cycle, highest priority only - stacking marks
  makes the tail unreadable
 */
bool CorvonGPS::draw_warning(uint32_t t)
{
    // both connectors powered. Miswired, and the mode that lost the
    // arbitration is not the one the operator expects
    if (dual_power) {
        const bool on = (t < 100) || (t >= 200 && t < 300);
        set_led(on ? 255 : 0, on ? 110 : 0, 0);        // orange double blink
        return true;
    }
#if AP_PERIPH_MAG_ENABLED
    // CAN mode only: in UART mode the compass sits on the host side of
    // the RS2058 and the MCU is not on that bus at all, so an absent
    // compass there says nothing
    if (mode == Mode::CAN &&
        (periph.compass.get_count() == 0 || !periph.compass.healthy())) {
        set_led(t < 250 ? 255 : 0, 0, 0);              // red single flash
        return true;
    }
#endif
    // CAN mode with nothing at all on the bus. An ArduPilot node
    // broadcasts NodeStatus every second, so 3s of silence means the
    // bus is unplugged or we are the only node on it
    if (mode == Mode::CAN &&
        (last_can_rx_ms == 0 || AP_HAL::millis() - last_can_rx_ms > 3000)) {
        const uint8_t v = (t < 250) ? 255 : 0;
        set_led(v, 0, v);                              // magenta single flash
        return true;
    }
    return false;
}

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
            // phase off the start of the test, not off the millis
            // counter. Anchored to absolute time the sweep opens on
            // whatever colour it happens to land on, and an operator
            // told to expect red-green-blue-white reads that as a fault
            const uint32_t t = now - (test_end_ms - CORVON_SELFTEST_MS);
            switch ((t / 500) % 4) {
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

    if (boot_ms == 0) {
        // anchor the boot sequence where the LED can actually be driven.
        // init() runs ahead of gps/compass/baro/imu init and that work
        // takes long enough to swallow the first step of the sweep - on
        // a DW609 the red step never reached the LED at all. The GNSS
        // grace window is better measured from here too: it is about how
        // long we have been listening, not how long the board has been on
        boot_ms = now;
    }
    const uint32_t since_boot = now - boot_ms;

    if (since_boot < CORVON_BOOT_SEQ_MS) {
        boot_pattern(since_boot);
        return;
    }

    // recomputed every call rather than on a slot tick, so the breath
    // is smooth at the 50Hz update rate
    const uint32_t cyc = since_boot % CORVON_CYCLE_MS;
    const uint8_t lvl = breathe_level(cyc);

    // nothing parsed from the receiver yet. Below the grace period the
    // receiver is simply still booting; past it the link is broken,
    // which is a different problem from "no fix" and must not share a
    // pattern with it
    const uint32_t grace_end = CORVON_BOOT_SEQ_MS + CORVON_GNSS_GRACE_MS;
    if (periph.gps.last_message_time_ms() == 0) {
        if (since_boot < grace_end) {
            set_led_dim(255, 255, 255, lvl / 2);       // dim white breathing
        } else {
            // phase the burst off the instant the link is declared dead
            // rather than off boot. grace_end lands 900ms into the cycle,
            // past the 600ms flash window, so a boot-anchored phase hides
            // the first burst for 1.1s - long enough to read as a dead LED
            flash_n((since_boot - grace_end) % CORVON_CYCLE_MS,
                    3, 255, 0, 0);                     // red triple flash
        }
        return;
    }

    // warning marks never cover a fault state, only healthy ones
    if (cyc >= CORVON_CYCLE_MS - CORVON_WARN_MS &&
        draw_warning(cyc - (CORVON_CYCLE_MS - CORVON_WARN_MS))) {
        return;
    }

    switch (periph.gps.status()) {
    case AP_GPS::NO_GPS:
    case AP_GPS::NO_FIX:
        // searching
        set_led_dim(0, 0, 255, lvl);
        break;
    case AP_GPS::GPS_OK_FIX_2D:
        set_led_dim(0, 255, 0, lvl);
        break;
    case AP_GPS::GPS_OK_FIX_3D:
    case AP_GPS::GPS_OK_FIX_3D_DGPS:
    case AP_GPS::GPS_OK_FIX_TYPE_STATIC:
    case AP_GPS::GPS_OK_FIX_TYPE_PPP:
        // usable fix: solid, the only steady state in the vocabulary
        set_led(0, 255, 0);
        break;
    case AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT:
        // kept for a future F9-class board; M9/F10 always report
        // carrSoln 0 so these two never trigger on G1/G2
        if (cyc < 150) {
            set_led(0, 0, 255);
        } else {
            set_led(0, 255, 0);
        }
        break;
    case AP_GPS::GPS_OK_FIX_3D_RTK_FIXED:
        if (cyc < 150) {
            set_led(255, 255, 255);
        } else {
            set_led(0, 255, 0);
        }
        break;
    }
}

#endif // HAL_CORVON_GPS_ENABLED
