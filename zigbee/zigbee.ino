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

#if !defined(ZIGBEE_MODE_ED) && !defined(ZIGBEE_MODE_ZCZR)
#error "Zigbee device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"
#include "esp_zigbee_core.h"

#ifdef ZIGBEE_MODE_ZCZR
zigbee_role_t role = ZIGBEE_COORDINATOR; //ZIGBEE_ROUTER;  // or can be ZIGBEE_COORDINATOR, but it won't scan itself
#else
zigbee_role_t role = ZIGBEE_END_DEVICE;
#endif


namespace ZIGBEE {
    static uint32_t pairing_start_time = 0;
    // Global variables to store valve data for I2C requests
    RTC_DATA_ATTR float currentTemp = 0;
    RTC_DATA_ATTR bool scan_in_progress = false;
    ZigbeeGateway zbGw = ZigbeeGateway(5);  // Endpoint 5 for switch

    void init() {
        LOG("[ZIGBEE] Initializing Zigbee stack as %s...", role == ZIGBEE_COORDINATOR ? "COORDINATOR" : (role == ZIGBEE_ROUTER ? "ROUTER" : "END DEVICE"));
        zbGw.setManufacturerAndModel(DEFAULT_HOSTNAME, "zigbee-switch");
        zbGw.allowMultipleBinding(true);
        Zigbee.addEndpoint(&zbGw);
        Zigbee.setRebootOpenNetwork(180);
        if(!Zigbee.begin(role)) {
            LOG("[ZIGBEE] Failed to initialize Zigbee stack");
            return;
        }
        LOG("[ZIGBEE] Zigbee stack initialized successfully");
    }

    void enable_pairing(uint32_t timeout_ms) {
        pairing_start_time = millis();
        LOG("[ZIGBEE] pairing enabled for %u ms", timeout_ms);
    }

    void disable_pairing() {
        LOG("[ZIGBEE] pairing disabled");
    }

    void loop() {
        // Check if pairing mode should be disabled after timeout
        if (pairing_start_time > 0 && (millis() - pairing_start_time) > 60000) {
            disable_pairing();
            pairing_start_time = 0;
        }
        if(scan_in_progress) {
            int16_t zigbee_scan_status = Zigbee.scanComplete();
            if (zigbee_scan_status < 0) {
                if(zigbee_scan_status == ZB_SCAN_FAILED){
                    LOG("[ZIGBEE] Scan failed with error code: %d", zigbee_scan_status);
                    scan_in_progress = false;
                }
            } else {
                scan_in_progress = false;
                zigbee_scan_result_t *scan_result = Zigbee.getScanResult();
                LOG("[ZIGBEE] Scan complete, found %d endpoints", zigbee_scan_status);
                if (zigbee_scan_status > 0 && scan_result != NULL) {
                    for (int i = 0; i < zigbee_scan_status; i++) {
                        LOG("[ZIGBEE] Network %d: PAN ID=0x%04X, Channel=%d, Permit Joining=%s, Router Capacity=%s, End Device Capacity=%s",
                            i + 1,
                            scan_result[i].short_pan_id,
                            scan_result[i].logic_channel,
                            scan_result[i].permit_joining      ? "Yes" : "No",
                            scan_result[i].router_capacity     ? "Yes" : "No",
                            scan_result[i].end_device_capacity ? "Yes" : "No"
                        );
                        LOG("[ZIGBEE] Extended PAN ID: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                            scan_result[i].extended_pan_id[7], scan_result[i].extended_pan_id[6], scan_result[i].extended_pan_id[5], scan_result[i].extended_pan_id[4],
                            scan_result[i].extended_pan_id[3], scan_result[i].extended_pan_id[2], scan_result[i].extended_pan_id[1], scan_result[i].extended_pan_id[0]
                        );
                    }
                }
                if(role == ZIGBEE_END_DEVICE) {
                    // Only delete scan results if we're an end device
                    // Calling this as coordinator/router causes a crash
                    Zigbee.scanDelete();
                }
            }
        }
        return;
    }

    std::list<zb_device_params_t *> get_bound_eps() {
        return zbGw.getBoundDevices();
    }

    void scan_eps() {
        // End devices: scan for networks to join
        LOG("[ZIGBEE] Scanning for Zigbee networks...");
        Zigbee.scanNetworks(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK, 5);
        scan_in_progress = true;
    }
}

namespace PLUGINS {
    void initialize() {
        ZIGBEE::init();
    }

    void loop_pre() {
        ZIGBEE::loop();
    }

    const char * at_cmd_handler(const char *at_cmd) {
        unsigned int cmd_len = strlen(at_cmd);
        ALIGN(4) static char response[512];
        char *p = NULL;
        errno = 0;
        D("[ZIGBEE] AT command received: %s", at_cmd);

        // AT+ZBLIST? - List all paired Zigbee devices
        if (p = COMMON::at_cmd_check("AT+ZBLIST?", at_cmd, cmd_len)) {
            size_t offset = 0;
            // Query list of paired devices
            LOG("[ZIGBEE] Listing devices bound to this network...");
            std::list<zb_device_params_t *> eps = ZIGBEE::get_bound_eps();
            // Coordinators/routers: list devices bound to our network
            LOG("[ZIGBEE] Found %d bound devices", eps.size());
            int i = 1;
            for (const auto &ep : eps) {
                LOG("[ZIGBEE] Device %d: IEEE=0x%016llX, Short=0x%04X, Endpoint=%d", 
                    i++, ep->ieee_addr, ep->short_addr, ep->endpoint);

                // Try to read Basic cluster attributes for device info
                // Cluster 0x0000 (Basic), Attribute 0x0004 (Manufacturer Name)
                esp_zb_zcl_read_attr_cmd_t read_req;
                read_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
                read_req.zcl_basic_cmd.dst_addr_u.addr_short = ep->short_addr;
                read_req.zcl_basic_cmd.dst_endpoint = ep->endpoint;
                read_req.zcl_basic_cmd.src_endpoint = ZIGBEE::zbGw.getEndpoint();
                read_req.clusterID = 0x0000; // Basic cluster

                LOG("[ZIGBEE] Requesting manufacturer name from device...");
                esp_zb_zcl_read_attr_cmd_req(&read_req);

                // Also request model identifier (0x0005)
                LOG("[ZIGBEE] Requesting model identifier from device...");
                esp_zb_zcl_read_attr_cmd_req(&read_req);

                // Append device info to response string
                offset += snprintf(response + offset, sizeof(response) - offset,
                    "+ZBLIST:IEEE=0x%016llX,Endpoint=%d\r\n", ep->ieee_addr, ep->endpoint);
                if (offset >= sizeof(response)) {
                    break; // Prevent buffer overflow
                }
            }
            if(eps.size() == 0) {
                snprintf(response, sizeof(response), "+ZBLIST:No devices found\r\n");
            }
            return response;
        }
        // AT+ZBJOIN=<timeout_sec> - Enable/disable permit joining
        else if (p = COMMON::at_cmd_check("AT+ZBJOIN=", at_cmd, cmd_len)) {
            // For coordinators/routers, "scan" means enabling permit joining
            // to allow new devices to join the network
            int timeout_sec = 0;
            if (sscanf(p, "%d", &timeout_sec) >= 1) {
                if (timeout_sec > 0) {
                    LOG("[ZIGBEE] Enabling permit joining for %d seconds", timeout_sec);
                    Zigbee.openNetwork(timeout_sec);
                    snprintf(response, sizeof(response), "+ZBJOIN:permit_join_enabled,%d\r\nOK", timeout_sec);
                } else {
                    LOG("[ZIGBEE] Disabling permit joining");
                    Zigbee.closeNetwork();
                    snprintf(response, sizeof(response), "+ZBJOIN:permit_join_disabled\r\nOK");
                }
                return response;
            }
            return AT_R("ERROR:INVALID_PARAM");
        }
        // AT+ZBSCAN? - Query scan status
        else if (p = COMMON::at_cmd_check("AT+ZBSCAN=", at_cmd, cmd_len)) {
            if (strcmp(p, "1") == 0) {
                ZIGBEE::scan_eps();
                return AT_R("OK");
            } else if (strcmp(p, "0") == 0) {
                return AT_R("ERROR:NOT_IMPLEMENTED");
            }
            return AT_R("ERROR:INVALID_PARAM");
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
