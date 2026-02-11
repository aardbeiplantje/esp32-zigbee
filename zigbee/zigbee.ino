/*
 * zigbee.ino: Common source file for Zigbee setup
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

#include <zigbee.h>
#include <common.h>
#include <string.h>

namespace ZIGBEE {
    // Global coordinator state
    static CoordinatorState coordinator = {
        .initialized = false,
        .pairing_enabled = false,
        .pairing_timeout = 0,
        .devices = std::vector<Device>()
    };

    static uint32_t pairing_start_time = 0;

    void init() {
        if (!coordinator.initialized) {
            coordinator.devices.clear();
            coordinator.pairing_enabled = false;
            coordinator.pairing_timeout = 0;
            coordinator.initialized = true;
            LOG("[ZIGBEE] coordinator initialized");
        }
    }

    void enable_pairing(uint32_t timeout_ms) {
        coordinator.pairing_enabled = true;
        coordinator.pairing_timeout = timeout_ms;
        pairing_start_time = millis();
        LOG("[ZIGBEE] pairing enabled for %u ms", timeout_ms);
    }

    void disable_pairing() {
        coordinator.pairing_enabled = false;
        coordinator.pairing_timeout = 0;
        LOG("[ZIGBEE] pairing disabled");
    }

    bool is_pairing_enabled() {
        if (coordinator.pairing_enabled) {
            // Check if timeout has expired
            if (coordinator.pairing_timeout > 0) {
                if (millis() - pairing_start_time >= coordinator.pairing_timeout) {
                    disable_pairing();
                    return false;
                }
            }
            return true;
        }
        return false;
    }

    const std::vector<Device>& get_devices() {
        return coordinator.devices;
    }

    bool add_device(const Device& dev) {
        // Check if device already exists
        for (auto& d : coordinator.devices) {
            if (d.ieee_addr == dev.ieee_addr) {
                // Update existing device
                d = dev;
                LOG("[ZIGBEE] device updated: 0x%016llX", dev.ieee_addr);
                return true;
            }
        }
        // Add new device
        coordinator.devices.push_back(dev);
        LOG("[ZIGBEE] device added: 0x%016llX", dev.ieee_addr);
        return true;
    }

    bool remove_device(uint64_t ieee_addr) {
        for (auto it = coordinator.devices.begin(); it != coordinator.devices.end(); ++it) {
            if (it->ieee_addr == ieee_addr) {
                LOG("[ZIGBEE] device removed: 0x%016llX", ieee_addr);
                coordinator.devices.erase(it);
                return true;
            }
        }
        return false;
    }

    Device* find_device(uint64_t ieee_addr) {
        for (auto& d : coordinator.devices) {
            if (d.ieee_addr == ieee_addr) {
                return &d;
            }
        }
        return NULL;
    }
}

namespace PLUGINS {
    void initialize() {
        ZIGBEE::init();
    }

    void setup() {
        // Additional setup if needed
    }

    const char * at_cmd_handler(const char *at_cmd) {
        unsigned int cmd_len = strlen(at_cmd);
        ALIGN(4) static char response[512];
        char *p = NULL;
        errno = 0;
        D("[ZIGBEE] AT command received: %s", at_cmd);

        // AT+ZBLIST? - List all paired Zigbee devices
        if (p = COMMON::at_cmd_check("AT+ZBLIST?", at_cmd, cmd_len)) {
            const auto& devices = ZIGBEE::get_devices();
            if (devices.empty()) {
                snprintf(response, sizeof(response), "+ZBLIST:0\r\nOK");
                return response;
            }

            // Build response with device list
            int offset = 0;
            offset += snprintf(response + offset, sizeof(response) - offset, 
                              "+ZBLIST:%d\r\n", (int)devices.size());

            for (size_t i = 0; i < devices.size() && offset < sizeof(response) - 100; i++) {
                const auto& dev = devices[i];
                offset += snprintf(response + offset, sizeof(response) - offset,
                                  "+ZBLIST:%d,0x%016llX,0x%04X,%d,%s,%s\r\n",
                                  (int)i,
                                  dev.ieee_addr,
                                  dev.short_addr,
                                  dev.endpoint,
                                  dev.online ? "online" : "offline",
                                  dev.name);
            }
            offset += snprintf(response + offset, sizeof(response) - offset, "OK");
            return response;
        }
        // AT+ZBPAIR? - Query pairing mode status
        else if (p = COMMON::at_cmd_check("AT+ZBPAIR?", at_cmd, cmd_len)) {
            D("[ZIGBEE] AT+ZBPAIR command detected, next: %s", p);
            // Query current pairing status
            bool enabled = ZIGBEE::is_pairing_enabled();
            snprintf(response, sizeof(response), "+ZBPAIR:%s\r\nOK",
                    enabled ? "enabled" : "disabled");
            return response;
        }
        // AT+ZBPAIR=<enable>[,<timeout_sec>] - Set pairing mode
        // AT+ZBPAIR=1,120 - Enable pairing for 120 seconds
        // AT+ZBPAIR=0 - Disable pairing
        else if (p = COMMON::at_cmd_check("AT+ZBPAIR=", at_cmd, cmd_len)) {
            int enable = 0;
            int timeout_sec = 60;  // Default 60 seconds

            // Parse enable parameter
            if (sscanf(p, "%d", &enable) >= 1) {
                // Check for optional timeout parameter
                char *comma = strchr(p, ',');
                if (comma != NULL) {
                    sscanf(comma + 1, "%d", &timeout_sec);
                }

                if (enable) {
                    ZIGBEE::enable_pairing(timeout_sec * 1000);
                    snprintf(response, sizeof(response), 
                            "+ZBPAIR:enabled,%d\r\nOK", timeout_sec);
                } else {
                    ZIGBEE::disable_pairing();
                    snprintf(response, sizeof(response), "+ZBPAIR:disabled\r\nOK");
                }
                return response;
            }
            return AT_R("ERROR");
        }
        // AT+ZBREM - Remove a device by IEEE address
        // AT+ZBREM=<ieee_addr>
        else if (p = COMMON::at_cmd_check("AT+ZBREM=", at_cmd, cmd_len)) {
            unsigned long long ieee_addr = 0;
            if (sscanf(p, "0x%llX", &ieee_addr) == 1 || sscanf(p, "%llu", &ieee_addr) == 1) {
                if (ZIGBEE::remove_device(ieee_addr)) {
                    return AT_R_OK;
                } else {
                    return AT_R("ERROR:DEVICE_NOT_FOUND");
                }
            }
            return AT_R("ERROR:INVALID_ADDR");
        }
        return NULL;  // Command not handled
    }

    const char * at_get_help_string() {
        return R"EOF(
Zigbee AT Commands:
  AT+ZBLIST?           - List all paired Zigbee devices
  AT+ZBREM=<ieee_addr> - Remove device by IEEE address
  AT+ZBPAIR?           - Query pairing mode status
  AT+ZBPAIR=<enable>[,<timeout_sec>] - Enable/disable pairing
                                        (0=disable, 1=enable)
                                        timeout_sec is optional (default 60s)
)EOF";
    }
}
