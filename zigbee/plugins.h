/*
 * plugins.h: Common header file for plugin setup
 *
 * Author: CowboyTim
 *
 * This is free and unencumbered software released into the public domain.
 *
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * In jurisdictions that recognize copyright laws, the author or authors
 * of this software dedicate any and all copyright interest in the
 * software to the public domain. We make this dedication for the benefit
 * of the public at large and to the detriment of our heirs and
 * successors. We intend this dedication to be an overt act of
 * relinquishment in perpetuity of all present and future rights to this
 * software under copyright law.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * For more information, please refer to <https://unlicense.org>
 */

#ifndef _PLUGINS_H
#define _PLUGINS_H

// Colors the status LED can show. A plugin returns one of these from
// led_idle_color() to show its own state while nothing else needs the LED.
#define LED_RGB_LEVEL 26 // 0-255 per channel, ~10% brightness to save power
#define LED_RGB_IDLE_LEVEL 4 // steady "running, nothing to report" glow, ~1.5%
enum { LED_RGB_NONE = 0, LED_RGB_RED, LED_RGB_GREEN, LED_RGB_BLUE, LED_RGB_MAGENTA, LED_RGB_CYAN,
       LED_RGB_IDLE,          // dim blue: running, nothing to report
       LED_RGB_IDLE_PURPLE,   // dim purple: the same, on a board you want to tell apart
       LED_RGB_ORANGE };      // bright orange: clearly not the dim blue idle glow

namespace PLUGINS {
    // add all functions as weak
    void initialize() __attribute__((weak));
    void setup() __attribute__((weak));
    void loop_pre() __attribute__((weak));
    void loop_post() __attribute__((weak));
    long max_sleep_time() __attribute__((weak));
    void clear_config() __attribute__((weak));
    const char * at_cmd_handler(const char *at_cmd) __attribute__((weak));
    const char * at_get_help_string() __attribute__((weak));
    // The color to show while nothing else needs the LED: an LED_RGB_* value,
    // or < 0 to leave the idle state alone.
    int led_idle_color() __attribute__((weak));
    // A long press (2-5 s) on the button that nothing else handled
    void button_long_press() __attribute__((weak));
    // The button has been held for 5 s (called while it is still held). With
    // this hook, holding the button first starts BLE (after 1 s), then at 5 s
    // stops BLE and calls it. Without it, a 5 s press keeps its old meaning.
    void button_hold_5s() __attribute__((weak));
    // A state that outranks BLE on the LED: return an LED_RGB_* color and set
    // *interval_ms (0 = steady), or return < 0 to leave the LED alone.
    int led_status(int *interval_ms) __attribute__((weak));
}

#endif // _PLUGINS_H
