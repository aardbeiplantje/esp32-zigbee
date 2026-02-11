/*
 * zigbee.h: Common header file for Zigbee setup
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

#ifndef _ZIGBEE_H
#define _ZIGBEE_H

#define SUPPORT_PLUGINS
#define SUPPORT_ESP_LOG_INFO
#define UART_AT
#define BLUETOOTH_UART_AT
#define LOOP_DELAY

#undef LOGUART
#undef TIMELOG
#undef SUPPORT_GPIO

#ifndef DEFAULT_HOSTNAME
#define DEFAULT_HOSTNAME "zigbee"
#endif

#include <vector>
#include <cstdint>

namespace PLUGINS {
    // Zigbee plugin API implementation
    void initialize();
    void setup();
    const char * at_cmd_handler(const char *at_cmd);
    const char * at_get_help_string();
}

namespace ZIGBEE {
    // Zigbee device structure
    struct Device {
        uint64_t ieee_addr;     // IEEE address (64-bit)
        uint16_t short_addr;    // Short address (16-bit)
        uint8_t endpoint;       // Endpoint
        bool online;            // Device status
        char name[32];          // Device name/identifier
    };

    // Zigbee coordinator state
    struct CoordinatorState {
        bool initialized;
        bool pairing_enabled;
        uint32_t pairing_timeout;
        std::vector<Device> devices;
    };

    // Core functions
    void init();
    void enable_pairing(uint32_t timeout_ms = 60000);
    void disable_pairing();
    bool is_pairing_enabled();
    const std::vector<Device>& get_devices();
    bool add_device(const Device& dev);
    bool remove_device(uint64_t ieee_addr);
    Device* find_device(uint64_t ieee_addr);
}

#endif // _ZIGBEE_H
