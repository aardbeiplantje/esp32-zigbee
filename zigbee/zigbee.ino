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

// Zigbee Coordinator mode is set via compiler flags from deploy.py: -DZIGBEE_MODE_ZCZR
#include <zigbee.h>
#include <common.h>

#if !defined(ZIGBEE_MODE_ED) && !defined(ZIGBEE_MODE_ZCZR)
#error "Zigbee device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"
#include "esp_zigbee_core.h"
#include <vector>

#ifdef ZIGBEE_MODE_ZCZR
#pragma message "ZIGBEE_MODE_ZCZR IS DEFINED - Using COORDINATOR (role 0)"
zigbee_role_t role = ZIGBEE_COORDINATOR;  // Confusing but COORDINATOR=0 in Arduino lib
#else
#pragma message "ZIGBEE_MODE_ZCZR NOT DEFINED - Using END_DEVICE (role 2)"
zigbee_role_t role = ZIGBEE_END_DEVICE;  // Confusing but END_DEVICE=2 in Arduino lib
#endif

namespace ZIGBEE {
    // Plain RAM, not RTC: these are millis()-based or per-session state, and
    // RTC memory survives software resets while millis() restarts at zero.
    bool scan_in_progress = false;
    bool device_discovery_in_progress = false;
    unsigned long permit_join_end_time = 0;
    unsigned long last_permit_join_log_time = 0;
    ZigbeeGateway zbGw = ZigbeeGateway(1);
    ZigbeeSwitch zbSwitch = ZigbeeSwitch(2);
    ZigbeeLight zbLight = ZigbeeLight(3);
    ZigbeeThermostat zbThermostat = ZigbeeThermostat(4);

    // defined below, used by loop()
    size_t count_children();

    // The coordinator is always short address 0x0000; our light is endpoint 3.
    #define ZB_COORDINATOR_SHORT_ADDR 0x0000
    #define ZB_LIGHT_ENDPOINT 3

    // State of our own light endpoint, shown on the status LED
    bool light_on = false;

    // Send an On/Off "toggle" to a light endpoint on another device.
    // ZigbeeSwitch::lightToggle() refuses unless the switch is bound, so this
    // builds the same ZCL command the Arduino method maps to and sends it
    // directly, which works for any device on the network.
    bool send_light_toggle(uint16_t short_addr, uint8_t dst_endpoint) {
        if(!Zigbee.started()) {
            LOG("[ZIGBEE] Cannot send a toggle: the Zigbee stack is not started");
            return false;
        }
        esp_zb_zcl_on_off_cmd_t cmd_req;
        memset(&cmd_req, 0, sizeof(cmd_req));
        cmd_req.zcl_basic_cmd.src_endpoint     = zbSwitch.getEndpoint();
        cmd_req.zcl_basic_cmd.dst_endpoint     = dst_endpoint;
        cmd_req.zcl_basic_cmd.dst_addr_u.addr_short = short_addr;
        cmd_req.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        cmd_req.on_off_cmd_id = ESP_ZB_ZCL_CMD_ON_OFF_TOGGLE_ID;
        if(!esp_zb_lock_acquire(pdMS_TO_TICKS(1000))) {
            LOG("[ZIGBEE] Could not acquire the Zigbee lock to send a toggle");
            return false;
        }
        esp_zb_zcl_on_off_cmd_req(&cmd_req);
        esp_zb_lock_release();
        LOG("[ZIGBEE] Sent light toggle to 0x%04X endpoint %d", short_addr, dst_endpoint);
        return true;
    }

    // A long press (2-5 s) on the BOOT button toggles a light: on an end device
    // the coordinator's, on the coordinator its own.
    void button_toggle_light() {
#if defined(ZIGBEE_MODE_ED)
        LOG("[ZIGBEE] Button: toggling the light on the coordinator");
        send_light_toggle(ZB_COORDINATOR_SHORT_ADDR, ZB_LIGHT_ENDPOINT);
#else
        LOG("[ZIGBEE] Button: toggling our own light");
        zbLight.setLight(!zbLight.getLightState());
#endif
    }

    // Information about a bound device, as reported by the Zigbee stack
    struct DiscoveredDevice {
        uint64_t ieee_addr;
        uint16_t short_addr;
        uint8_t endpoint;
    };

    std::vector<DiscoveredDevice> discovered_devices;

    void init() {
        LOG("[ZIGBEE] Initializing Zigbee stack as %s...", role == ZIGBEE_COORDINATOR ? "COORDINATOR" : (role == ZIGBEE_ROUTER ? "ROUTER" : "END DEVICE"));
        zbGw.setManufacturerAndModel(DEFAULT_HOSTNAME, "zigbee-gateway");
        zbGw.setPowerSource(ZB_POWER_SOURCE_MAINS);
        zbGw.allowMultipleBinding(true);
        zbGw.onIdentify([](uint16_t duration) {
            LOG("[ZIGBEE] Identify command received with duration %d seconds", duration);
        });
        zbGw.onDefaultResponse([](zb_cmd_type_t resp_to_cmd, esp_zb_zcl_status_t status) {
            LOG("[ZIGBEE] Default response received for command 0x%02X with status 0x%02X", resp_to_cmd, status);
        });
        LOG("[ZIGBEE] Adding gateway endpoint with ID %d", zbGw.getEndpoint());
        Zigbee.addEndpoint(&zbGw);

        zbSwitch.setManufacturerAndModel(DEFAULT_HOSTNAME, "zigbee-switch");
        zbSwitch.setPowerSource(ZB_POWER_SOURCE_MAINS);
        zbSwitch.allowMultipleBinding(true);
        zbSwitch.onDefaultResponse([](zb_cmd_type_t resp_to_cmd, esp_zb_zcl_status_t status) {
            LOG("[ZIGBEE] Switch default response received for command 0x%02X with status 0x%02X", resp_to_cmd, status);
        });
        zbSwitch.onLightStateChangeWithSource([](bool on, uint8_t src, esp_zb_zcl_addr_t src_addr) {
            LOG("[ZIGBEE] Switch light state changed to %s", on ? "ON" : "OFF");
        });
        zbSwitch.onIdentify([](uint16_t duration) {
            LOG("[ZIGBEE] Switch Identify command received with duration %d seconds", duration);
        });
        LOG("[ZIGBEE] Adding switch endpoint with ID %d", zbSwitch.getEndpoint());
        Zigbee.addEndpoint(&zbSwitch);

        zbLight.setManufacturerAndModel(DEFAULT_HOSTNAME, "zigbee-light");
        zbLight.setPowerSource(ZB_POWER_SOURCE_MAINS);
        zbLight.allowMultipleBinding(true);
        zbLight.onDefaultResponse([](zb_cmd_type_t resp_to_cmd, esp_zb_zcl_status_t status) {
            LOG("[ZIGBEE] Light default response received for command 0x%02X with status 0x%02X", resp_to_cmd, status);
        });
        zbLight.onIdentify([](uint16_t duration) {
            LOG("[ZIGBEE] Light Identify command received with duration %d seconds", duration);
        });
        zbLight.onLightChange([](bool on) {
            // Someone on the network switched our light: show it on the status LED
            light_on = on;
            LOG("[ZIGBEE] Light switched %s", on ? "ON" : "OFF");
        });
        LOG("[ZIGBEE] Adding light endpoint with ID %d", zbLight.getEndpoint());
        Zigbee.addEndpoint(&zbLight);

        zbThermostat.setManufacturerAndModel(DEFAULT_HOSTNAME, "zigbee-thermostat");
        zbThermostat.setPowerSource(ZB_POWER_SOURCE_MAINS);
        zbThermostat.allowMultipleBinding(true);
        zbThermostat.onIdentify([](uint16_t duration) {
            LOG("[ZIGBEE] Thermostat Identify command received with duration %d seconds", duration);
        });
        LOG("[ZIGBEE] Adding thermostat endpoint with ID %d", zbThermostat.getEndpoint());
        Zigbee.addEndpoint(&zbThermostat);

#if defined(ZIGBEE_MODE_ED)
        // End device: it joins a network that is open for joining; it does not open one itself
        LOG("[ZIGBEE] End device: joining a network (open it on the coordinator with AT+ZBJOIN=<sec>)");
#else
        // wait for pairing/joining during 180s after reboot
        LOG("[ZIGBEE] Setting open network for 180 seconds after reboot to allow joining");
        Zigbee.setRebootOpenNetwork(180);
#endif
        LOG("[ZIGBEE] About to call Zigbee.begin() with role=%d  (ZIGBEE_COORDINATOR=%d, ZIGBEE_END_DEVICE=%d)", role, ZIGBEE_COORDINATOR, ZIGBEE_END_DEVICE);
        if(!Zigbee.begin(role)) {
            LOG("[ZIGBEE] Failed to initialize Zigbee stack");
            return;
        }
        LOG("[ZIGBEE] Zigbee stack initialized successfully");
#if defined(ZIGBEE_MODE_ED)
        LOG("[ZIGBEE] *** ESP32H2 is an END DEVICE - waiting to join a network ***");
#else
        LOG("[ZIGBEE] *** ESP32H2 is now the COORDINATOR - network ready for devices to join ***");
        LOG("[ZIGBEE] Enable pairing with: AT+ZBJOIN=60");
#endif
        // Log network diagnostics
        delay(100);  // Give stack time to settle
        LOG("[ZIGBEE] Stack state check: connected=%s", Zigbee.connected() ? "YES" : "NO");
    }

    void loop() {
#if defined(ZIGBEE_MODE_ED)
        // log each change of the join state (the stack joins in the background)
        static int8_t last_connected = -1;
        if(Zigbee.connected() != (last_connected == 1)) {
            last_connected = Zigbee.connected() ? 1 : 0;
            LOG("[ZIGBEE] Network state: %s", last_connected ? "JOINED" : "NOT JOINED");
        }
#endif
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

        // Monitor permit join status
        if (permit_join_end_time > 0 && millis() < permit_join_end_time) {
            unsigned long now = millis();
            unsigned long remaining = (permit_join_end_time - now) / 1000;
            // Log every 10 seconds (only when 10+ seconds have passed since last log)
            if (now - last_permit_join_log_time >= 10000) {
                LOG("[ZIGBEE] *** Permit Joining Active *** %lu seconds remaining, %d devices on the network", remaining, count_children());
                if (last_permit_join_log_time == 0) {
                    // First log - show network diagnostics
                    LOG("[ZIGBEE] Network state: Zigbee connected=%s, permit_join_end_time=%lu", 
                        Zigbee.connected() ? "YES" : "NO", permit_join_end_time);
                }
                last_permit_join_log_time = now;
            }
        } else if (permit_join_end_time > 0) {
            permit_join_end_time = 0;
            last_permit_join_log_time = 0;
            LOG("[ZIGBEE] Permit joining window closed - devices on the network: %d", count_children());
        }
    }

    void scan_eps() {
        // End devices: scan for networks to join
        LOG("[ZIGBEE] Scanning for Zigbee networks...");
        Zigbee.scanNetworks(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK, 5);
        scan_in_progress = true;
    }

    void discover_devices() {
        // List the devices bound to this network (no ZCL requests: those cannot
        // be made safely from the AT command handler context)
        LOG("[ZIGBEE] Starting device discovery...");
        device_discovery_in_progress = true;
        discovered_devices.clear();

        std::list<zb_device_params_t *> eps = zbGw.getBoundDevices();
        LOG("[ZIGBEE] Found %d devices to scan", eps.size());

        if (eps.size() == 0) {
            LOG("[ZIGBEE] No devices found on network");
            device_discovery_in_progress = false;
            return;
        }

        // For each device, collect basic information without making ZCL requests
        // ZCL requests from AT handler can cause critical section issues
        for (const auto &ep : eps) {
            if (!ep) {
                LOG("[ZIGBEE] Skipping NULL device pointer in bound devices list");
                continue;
            }
            // Create device entry
            DiscoveredDevice dev;
            // Convert IEEE address byte array to 64-bit integer
            dev.ieee_addr = 0;
            if (ep->ieee_addr) {
                for (int i = 0; i < 8; i++) {
                    dev.ieee_addr = (dev.ieee_addr << 8) | ep->ieee_addr[7 - i];
                }
            }
            dev.short_addr = ep->short_addr;
            dev.endpoint = ep->endpoint;

            discovered_devices.push_back(dev);
            LOG("[ZIGBEE] Found device: Short=0x%04X, Endpoint=%d", ep->short_addr, ep->endpoint);
        }

        device_discovery_in_progress = false;
        LOG("[ZIGBEE] Device discovery complete. Found %d devices", discovered_devices.size());
    }
    void get_discovered_devices(char *response, size_t resp_len) {
        // Format discovered devices into response string
        size_t offset = 0;

        if (discovered_devices.size() == 0) {
            snprintf(response, resp_len, "+ZBDEVICES:No devices found\r\n");
            return;
        }

        for (const auto &dev : discovered_devices) {
            offset += snprintf(response + offset, resp_len - offset,
                "+ZBDEVICES:IEEE=0x%016llX,Short=0x%04X,Endpoint=%d\r\n",
                dev.ieee_addr, dev.short_addr, dev.endpoint);

            if (offset >= resp_len) {
                LOG("[ZIGBEE] Device list response buffer near full, truncating");
                break;
            }
        }
    }

    // A device that joins the network shows up in the stack's neighbour table.
    // getBoundDevices() only knows devices that bound to one of our endpoints,
    // so a plain join is invisible there; the neighbour table is the real answer
    // to "did it join?".
    struct Neighbor {
        uint8_t ieee_addr[8];
        uint16_t short_addr;
        uint8_t device_type;
        uint8_t relationship;
        uint8_t lqi;
        int8_t rssi;
    };

    const char *nwk_relationship_name(uint8_t r) {
        switch(r) {
            case ESP_ZB_NWK_RELATIONSHIP_PARENT:                return "parent";
            case ESP_ZB_NWK_RELATIONSHIP_CHILD:                 return "child";
            case ESP_ZB_NWK_RELATIONSHIP_SIBLING:               return "sibling";
            case ESP_ZB_NWK_RELATIONSHIP_PREVIOUS_CHILD:        return "left";
            case ESP_ZB_NWK_RELATIONSHIP_UNAUTHENTICATED_CHILD: return "joining";
            default:                                            return "unknown";
        }
    }

    const char *nwk_device_type_name(uint8_t t) {
        switch(t) {
            case ESP_ZB_DEVICE_TYPE_COORDINATOR: return "coordinator";
            case ESP_ZB_DEVICE_TYPE_ROUTER:      return "router";
            case ESP_ZB_DEVICE_TYPE_ED:          return "end-device";
            default:                             return "unknown";
        }
    }

    // Copy the neighbour table out while the Zigbee lock is held; format it after.
    size_t get_neighbors(Neighbor *out, size_t max) {
        size_t n = 0;
        if(!Zigbee.started())
            return 0;
        if(!esp_zb_lock_acquire(pdMS_TO_TICKS(1000))) {
            LOG("[ZIGBEE] Could not acquire the Zigbee lock to read the neighbour table");
            return 0;
        }
        esp_zb_nwk_info_iterator_t it = ESP_ZB_NWK_INFO_ITERATOR_INIT;
        esp_zb_nwk_neighbor_info_t nbr;
        while(n < max && esp_zb_nwk_get_next_neighbor(&it, &nbr) == ESP_OK) {
            memcpy(out[n].ieee_addr, nbr.ieee_addr, 8);
            out[n].short_addr   = nbr.short_addr;
            out[n].device_type  = nbr.device_type;
            out[n].relationship = nbr.relationship;
            out[n].lqi          = nbr.lqi;
            out[n].rssi         = nbr.rssi;
            n++;
        }
        esp_zb_lock_release();
        return n;
    }

    // Devices that are on our network right now (children, and devices joining)
    size_t count_children() {
        Neighbor nbrs[16];
        size_t n = get_neighbors(nbrs, 16);
        size_t c = 0;
        for(size_t i = 0; i < n; i++)
            if(nbrs[i].relationship == ESP_ZB_NWK_RELATIONSHIP_CHILD
               || nbrs[i].relationship == ESP_ZB_NWK_RELATIONSHIP_UNAUTHENTICATED_CHILD)
                c++;
        return c;
    }

    // Append "+<prefix>:..." lines for the neighbour table, returns the new offset
    size_t append_neighbors(char *buf, size_t buf_len, size_t offset, const char *prefix) {
        Neighbor nbrs[16];
        size_t n = get_neighbors(nbrs, 16);
        LOG("[ZIGBEE] Neighbour table: %d entries", n);
        for(size_t i = 0; i < n && offset < buf_len; i++) {
            char ieee_str[24] = {0};
            snprintf(ieee_str, sizeof(ieee_str), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                nbrs[i].ieee_addr[7], nbrs[i].ieee_addr[6], nbrs[i].ieee_addr[5], nbrs[i].ieee_addr[4],
                nbrs[i].ieee_addr[3], nbrs[i].ieee_addr[2], nbrs[i].ieee_addr[1], nbrs[i].ieee_addr[0]);
            LOG("[ZIGBEE] Neighbour %d: IEEE=%s, Short=0x%04X, %s, %s, LQI=%d, RSSI=%d",
                i + 1, ieee_str, nbrs[i].short_addr, nwk_device_type_name(nbrs[i].device_type),
                nwk_relationship_name(nbrs[i].relationship), nbrs[i].lqi, nbrs[i].rssi);
            offset += snprintf(buf + offset, buf_len - offset,
                "+%s:IEEE=%s,Short=0x%04X,Type=%s,Rel=%s,LQI=%d,RSSI=%d\r\n",
                prefix, ieee_str, nbrs[i].short_addr, nwk_device_type_name(nbrs[i].device_type),
                nwk_relationship_name(nbrs[i].relationship), nbrs[i].lqi, nbrs[i].rssi);
        }
        return offset;
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
            std::list<zb_device_params_t *> eps = zbGw.getBoundDevices();
            // Coordinators/routers: list devices bound to our network
            LOG("[ZIGBEE] Found %d bound devices", eps.size());
            int i = 1;
            for (const auto &ep : eps) {
                if (!ep) {
                    LOG("[ZIGBEE] Skipping NULL device pointer in bound devices list");
                    continue;
                }
                // Format IEEE address from byte array
                char ieee_str[24] = {0};
                if (ep->ieee_addr) {
                    snprintf(ieee_str, sizeof(ieee_str), "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
                        ep->ieee_addr[7], ep->ieee_addr[6], ep->ieee_addr[5], ep->ieee_addr[4],
                        ep->ieee_addr[3], ep->ieee_addr[2], ep->ieee_addr[1], ep->ieee_addr[0]);
                }
                LOG("[ZIGBEE] Device %d: IEEE=%s, Short=0x%04X, Endpoint=%d", 
                    i, ieee_str, ep->short_addr, ep->endpoint);

                // Append device info to response string
                offset += snprintf(response + offset, sizeof(response) - offset,
                    "+ZBLIST:IEEE=%s,Short=0x%04X,Endpoint=%d\r\n", ieee_str, ep->short_addr, ep->endpoint);
                if (offset >= sizeof(response)) {
                    break; // Prevent buffer overflow
                }
                i++;
            }
            // Devices that joined but bound nothing are only in the neighbour table
            offset = append_neighbors(response, sizeof(response), offset, "ZBLIST");
            if(offset == 0) {
                snprintf(response, sizeof(response), "+ZBLIST:No devices found\r\n");
            }
            return response;
        }
        // AT+ZBNEIGHBORS? - List the stack's neighbour table (who is on our network)
        else if (p = COMMON::at_cmd_check("AT+ZBNEIGHBORS?", at_cmd, cmd_len)) {
            LOG("[ZIGBEE] Listing the neighbour table...");
            size_t offset = append_neighbors(response, sizeof(response), 0, "ZBNEIGHBOR");
            if(offset == 0) {
                snprintf(response, sizeof(response), "+ZBNEIGHBOR:No neighbours found\r\n");
            }
            return response;
        }
        // AT+ZBTOGGLE[=<short_addr>] - Toggle a light, the same as a long button press
        else if (p = COMMON::at_cmd_check("AT+ZBTOGGLE=", at_cmd, cmd_len)) {
            unsigned int short_addr = 0;
            if (sscanf(p, "%x", &short_addr) != 1) {
                return AT_R("+ERROR: Use AT+ZBTOGGLE=<short address in hex, e.g. 87FF>");
            }
            if (!send_light_toggle((uint16_t)short_addr, ZB_LIGHT_ENDPOINT)) {
                return AT_R("+ERROR: Could not send the toggle");
            }
            snprintf(response, sizeof(response), "+ZBTOGGLE:sent to 0x%04X endpoint %d\r\nOK",
                     (uint16_t)short_addr, ZB_LIGHT_ENDPOINT);
            return response;
        }
        else if (p = COMMON::at_cmd_check("AT+ZBTOGGLE", at_cmd, cmd_len)) {
            button_toggle_light();
            return AT_R("OK");
        }
        // AT+ZBLIGHT=<0|1> / AT+ZBLIGHT? - Our own light endpoint
        else if (p = COMMON::at_cmd_check("AT+ZBLIGHT=", at_cmd, cmd_len)) {
            if (*p != '0' && *p != '1') {
                return AT_R("+ERROR: Use AT+ZBLIGHT=1 to switch on, AT+ZBLIGHT=0 to switch off");
            }
            zbLight.setLight(*p == '1');
            return AT_R("OK");
        }
        else if (p = COMMON::at_cmd_check("AT+ZBLIGHT?", at_cmd, cmd_len)) {
            snprintf(response, sizeof(response), "+ZBLIGHT:%s", zbLight.getLightState() ? "on" : "off");
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
                    LOG("[ZIGBEE] *** PERMIT JOINING ENABLED - Put your devices in pairing mode NOW ***");
                    permit_join_end_time = millis() + (timeout_sec * 1000);
                    last_permit_join_log_time = 0;  // Reset to log immediately
                    Zigbee.openNetwork(timeout_sec);
                    LOG("[ZIGBEE] openNetwork(%d) called, waiting for join attempts...", timeout_sec);
                    snprintf(response, sizeof(response), "+ZBJOIN:permit_join_enabled,%d\r\nOK", timeout_sec);
                } else {
                    LOG("[ZIGBEE] Disabling permit joining");
                    permit_join_end_time = 0;
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
                scan_eps();
                return AT_R("OK");
            } else if (strcmp(p, "0") == 0) {
                return AT_R("ERROR:NOT_IMPLEMENTED");
            }
            return AT_R("ERROR:INVALID_PARAM");
        }
        // AT+ZBDEVICES=1 - Snapshot the bound devices for AT+ZBDEVICES?
        else if (p = COMMON::at_cmd_check("AT+ZBDEVICES=", at_cmd, cmd_len)) {
            if (strcmp(p, "1") == 0) {
                LOG("[ZIGBEE] Device discovery triggered via AT command");
                discover_devices();
                snprintf(response, sizeof(response), "OK");
                return response;
            }
            return AT_R("ERROR:INVALID_PARAM");
        }
        // AT+ZBDEVICES? - List discovered devices
        else if (p = COMMON::at_cmd_check("AT+ZBDEVICES?", at_cmd, cmd_len)) {
            if (device_discovery_in_progress) {
                return AT_R("+ZBDEVICES:Discovery in progress\r\n");
            }
            get_discovered_devices(response, sizeof(response));
            return response;
        }
        return NULL;  // Command not handled
    }

    const char * at_get_help_string() {
        return R"EOF(
Zigbee AT Commands:
  AT+ZBLIST?          - List paired devices and devices on the network
  AT+ZBNEIGHBORS?     - List the neighbour table (devices on our network)
  AT+ZBTOGGLE         - Toggle a light, as a long (2-5 s) button press does:
                        on an end device the coordinator's, else our own
  AT+ZBTOGGLE=<short> - Toggle the light on the device with that short address
  AT+ZBLIGHT=<0|1>    - Switch our own light endpoint on or off
  AT+ZBLIGHT?         - Get our own light state
  AT+ZBJOIN=<sec>     - Enable permit joining for specified seconds (0 to disable)
  AT+ZBSCAN=1         - Scan for nearby Zigbee networks (end devices)
  AT+ZBDEVICES=1      - Snapshot the devices bound to the network
  AT+ZBDEVICES?       - List the snapshot taken by AT+ZBDEVICES=1
)EOF";
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
        return ZIGBEE::at_cmd_handler(at_cmd);
    }
    const char * at_get_help_string() {
        return ZIGBEE::at_get_help_string();
    }
    int led_idle_color() {
        // Show the Zigbee light state while nothing else needs the LED
        if(ZIGBEE::light_on) {
            // orange, not green: against the dim yellow idle glow green is hard to tell apart
            return LED_RGB_ORANGE;
        }
#if defined(ZIGBEE_MODE_ED)
        // The end device is the board with the button: purple, so the two
        // boards are easy to tell apart. The coordinator keeps the dim yellow.
        return LED_RGB_IDLE_PURPLE;
#else
        return -1;
#endif
    }
    void button_long_press() {
        // Toggle the light on the coordinator (on the coordinator itself, our own)
        ZIGBEE::button_toggle_light();
    }
}
