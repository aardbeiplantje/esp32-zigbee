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
#include <vector>

#ifdef ZIGBEE_MODE_ZCZR
zigbee_role_t role = ZIGBEE_COORDINATOR; //ZIGBEE_ROUTER;  // or can be ZIGBEE_COORDINATOR, but it won't scan itself
#else
zigbee_role_t role = ZIGBEE_END_DEVICE;
#endif


namespace ZIGBEE {
    // Global variables to store valve data for I2C requests
    RTC_DATA_ATTR float currentTemp = 0;
    RTC_DATA_ATTR bool scan_in_progress = false;
    RTC_DATA_ATTR bool device_discovery_in_progress = false;
    RTC_DATA_ATTR unsigned long permit_join_end_time = 0;
    RTC_DATA_ATTR unsigned long last_permit_join_log_time = 0;
    ZigbeeGateway zbGw = ZigbeeGateway(1);
    ZigbeeSwitch zbSwitch = ZigbeeSwitch(2);
    ZigbeeLight zbLight = ZigbeeLight(3);
    ZigbeeThermostat zbThermostat = ZigbeeThermostat(4);

    // Structure to hold discovered device information
    struct DiscoveredDevice {
        uint64_t ieee_addr;
        uint16_t short_addr;
        uint8_t endpoint;
        bool is_trv;
        bool is_temp_sensor;
        bool has_thermostat_cluster;
        bool has_temp_cluster;
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

        // wait for pairing/joining during 180s after reboot
        LOG("[ZIGBEE] Setting open network for 180 seconds after reboot to allow joining");
        Zigbee.setRebootOpenNetwork(180);
        if(!Zigbee.begin(role)) {
            LOG("[ZIGBEE] Failed to initialize Zigbee stack");
            return;
        }
        LOG("[ZIGBEE] Zigbee stack initialized successfully");
        LOG("[ZIGBEE] *** ESP32H2 is now the COORDINATOR - network ready for devices to join ***");
        LOG("[ZIGBEE] Enable pairing with: AT+ZBJOIN=60");
    }

    void loop() {
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
                std::list<zb_device_params_t *> eps = zbGw.getBoundDevices();
                LOG("[ZIGBEE] *** Permit Joining Active *** %lu seconds remaining, %d devices connected", remaining, eps.size());
                last_permit_join_log_time = now;
            }
        } else if (permit_join_end_time > 0) {
            permit_join_end_time = 0;
            last_permit_join_log_time = 0;
            std::list<zb_device_params_t *> eps = zbGw.getBoundDevices();
            LOG("[ZIGBEE] Permit joining window closed - Final device count: %d", eps.size());
        }
        
        if(Zigbee.connected()) {
            // Only print connected status for coordinators/routers, end devices won't be able to connect until they join a network
            LOG("[ZIGBEE] Device is connected to a network");
        } else {
            return;
        }
        return;
    }

    void scan_eps() {
        // End devices: scan for networks to join
        LOG("[ZIGBEE] Scanning for Zigbee networks...");
        Zigbee.scanNetworks(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK, 5);
        scan_in_progress = true;
    }

    void discover_devices() {
        // Discover all devices on the network and identify TRVs and temperature sensors
        LOG("[ZIGBEE] Starting device discovery for TRVs and temperature sensors...");
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
            dev.is_trv = false;
            dev.is_temp_sensor = false;
            dev.has_thermostat_cluster = false;
            dev.has_temp_cluster = false;

            // Note: Actual cluster detection would require sending ZCL requests
            // which cannot be safely done from the AT command handler context
            // In a real implementation, cluster info would be cached during init
            // or discovered asynchronously through ZCL attribute responses

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
            const char *type_str = "";
            if (dev.is_trv && dev.is_temp_sensor) {
                type_str = "TRV+TEMP";
            } else if (dev.is_trv) {
                type_str = "TRV";
            } else if (dev.is_temp_sensor) {
                type_str = "TEMP";
            }

            offset += snprintf(response + offset, resp_len - offset,
                "+ZBDEVICES:IEEE=0x%016llX,Short=0x%04X,Endpoint=%d,Type=%s\r\n",
                dev.ieee_addr, dev.short_addr, dev.endpoint, type_str);

            if (offset >= resp_len) {
                LOG("[ZIGBEE] Device list response buffer near full, truncating");
                break;
            }
        }
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
                    LOG("[ZIGBEE] *** PERMIT JOINING ENABLED - Put your devices in pairing mode NOW ***");
                    permit_join_end_time = millis() + (timeout_sec * 1000);
                    Zigbee.openNetwork(timeout_sec);
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
        // AT+ZBDEVICES=1 - Discover TRVs and temperature sensors
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
  AT+ZBLIST?          - List all paired Zigbee devices
  AT+ZBJOIN=<sec>     - Enable permit joining for specified seconds (0 to disable)
  AT+ZBSCAN=1         - Scan for nearby Zigbee networks (end devices)
  AT+ZBDEVICES=1      - Discover and scan for TRVs and temperature sensors
  AT+ZBDEVICES?       - List discovered TRVs and temperature sensors
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
}
