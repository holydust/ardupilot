/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "SerialLED.h"

#if AP_NOTIFY_SERIALLED_ENABLED

extern const AP_HAL::HAL& hal;

SerialLED::SerialLED(uint8_t led_off, uint8_t led_bright, uint8_t led_medium, uint8_t led_dim) :
    RGBLed(led_off, led_bright, led_medium, led_dim)
{
}

bool SerialLED::init()
{
    enable_mask = init_ports();
    return true;
}

bool SerialLED::hw_set_rgb(uint8_t red, uint8_t green, uint8_t blue)
{
    if (enable_mask == 0) {
        // nothing is enabled, no pins set as LED output. That is decided
        // once in init() and never changes, so there is nothing to retry
        // and reporting success keeps RGBLed from polling a dead path
        return true;
    }

    AP_SerialLED *led = AP_SerialLED::get_singleton();
    if (led == nullptr) {
        return false;
    }

    // Report what actually happened. RCOutput refuses LED data while it is
    // still initialising, while serial_nleds is zero, while a send is
    // pending, and if the lazy DMA buffer setup on the first call fails -
    // all of which happen around boot. RGBLed::_set_rgb() latches the colour
    // as current whenever this returns true and then skips the write until
    // the colour changes, so claiming success for a dropped write loses that
    // colour entirely rather than for one cycle. Measured on a CORVON-G1 on
    // 2026-08-25: the whole 300ms red phase of the boot sequence never
    // reached the LED, and the first light out of it was the green at 300ms.
    bool ok = true;

    for (uint16_t chan=0; chan<16; chan++) {
        if ((1U<<chan) & enable_mask) {
            ok = led->set_RGB(chan+1, -1, red, green, blue) && ok;
        }
    }

    for (uint16_t chan=0; chan<16; chan++) {
        if ((1U<<chan) & enable_mask) {
            ok = led->send(chan+1) && ok;
        }
    }

    return ok;
}

#endif  // AP_NOTIFY_SERIALLED_ENABLED
