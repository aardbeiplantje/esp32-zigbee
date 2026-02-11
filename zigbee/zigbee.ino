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

#include <Zigbee.h>

namespace ZIGBEE {
    static uint32_t pairing_start_time = 0;

    void init() {
        Zigbee.begin(ZIGBEE_COORDINATOR);
        LOG("[ZIGBEE] coordinator initialized");
    }

    void enable_pairing(uint32_t timeout_ms) {
        pairing_start_time = millis();
        LOG("[ZIGBEE] pairing enabled for %u ms", timeout_ms);
    }

    void disable_pairing() {
        LOG("[ZIGBEE] pairing disabled");
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
            // Query list of paired devices
            // TODO: Implement actual device retrieval from Zigbee stack
            return AT_R("ERROR:NOT_IMPLEMENTED");
        }
        // AT+ZBPAIR? - Query pairing mode status
        else if (p = COMMON::at_cmd_check("AT+ZBPAIR?", at_cmd, cmd_len)) {
            D("[ZIGBEE] AT+ZBPAIR command detected, next: %s", p);
            // Check if pairing is currently enabled based on time
            // TODO: This should check actual pairing status from Zigbee stack
            return AT_R("ERROR:NOT_IMPLEMENTED");
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
                    snprintf(response, sizeof(response), "+ZBPAIR:enabled,%d\r\nOK", timeout_sec);
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
                // TODO: Remove device with this IEEE address
                return AT_R("ERROR:NOT_IMPLEMENTED");
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
