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

// Our thermostat endpoint. ZigbeeThermostat only handles temperature-measurement
// reads; a valve's answers come on its thermostat cluster (0x0201), handled here.
class CoordinatorThermostat : public ZigbeeThermostat {
public:
    using ZigbeeThermostat::ZigbeeThermostat;
    bool answered = false;          // the valve answered our last read or write
    float local_temp = NAN;         // the valve's own temperature reading, deg C
    float setpoint = NAN;           // the valve's heating setpoint, deg C
    uint8_t write_status = 0xFF;    // status of the last write, 0x00 = success

    void zbAttributeRead(uint16_t cluster_id, const esp_zb_zcl_attribute_t *attribute, uint8_t src_endpoint, esp_zb_zcl_addr_t src_address) override {
        if(cluster_id != ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT || !attribute->data.value)
            return;
        if(attribute->data.type != ESP_ZB_ZCL_ATTR_TYPE_S16)
            return;
        float c = *(int16_t *)attribute->data.value / 100.0f;   // thermostat values are in 0.01 deg C
        if(attribute->id == ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID) {
            local_temp = c;
        } else if(attribute->id == ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID) {
            setpoint = c;
        } else {
            return;
        }
        answered = true;
        LOG("[VALVE] 0x%04X: %s = %.2f C", src_address.u.short_addr,
            attribute->id == ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID ? "room temperature" : "setpoint", c);
    }

    void zbWriteAttributeResponse(uint16_t cluster_id, uint16_t attribute_id, esp_zb_zcl_status_t status, uint8_t src_endpoint, esp_zb_zcl_addr_t src_address) override {
        if(cluster_id != ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT)
            return;
        write_status = status;
        answered = true;
        LOG("[VALVE] 0x%04X: write of attribute 0x%04X status 0x%02X", src_address.u.short_addr, attribute_id, status);
    }
};

namespace ZIGBEE {
    // Plain RAM, not RTC: these are millis()-based or per-session state, and
    // RTC memory survives software resets while millis() restarts at zero.
    bool scan_in_progress = false;
    bool device_discovery_in_progress = false;
    unsigned long permit_join_end_time = 0;
    unsigned long last_permit_join_log_time = 0;
    ZigbeeGateway zbGw = ZigbeeGateway(1);
    ZigbeeSwitch zbSwitch = ZigbeeSwitch(2);
    ZigbeeDimmableLight zbLight = ZigbeeDimmableLight(3);
    CoordinatorThermostat zbThermostat = CoordinatorThermostat(4);

    // defined below, used by loop()
    size_t count_children();
    void log_children();

    // Join mode, for the LED: children when the window opened, and whether one joined since
    static size_t join_children_at_open = 0;
    static bool join_device_joined = false;

    void open_join_window(int timeout_sec) {
        LOG("[ZIGBEE] Enabling permit joining for %d seconds", timeout_sec);
        LOG("[ZIGBEE] *** PERMIT JOINING ENABLED - Put your devices in pairing mode NOW ***");
        join_children_at_open = count_children();
        join_device_joined = false;
        permit_join_end_time = millis() + (timeout_sec * 1000);
        last_permit_join_log_time = 0;  // Reset to log immediately
        // ESP-IDF rule: calls into the Zigbee stack from another task need the Zigbee lock.
        // Without it the stack's interrupt bookkeeping can go wrong (assert in vPortExitCritical).
        if(!esp_zb_lock_acquire(pdMS_TO_TICKS(1000))) {
            LOG("[ZIGBEE] Could not take the Zigbee lock to open the network");
            return;
        }
        Zigbee.openNetwork((uint8_t)timeout_sec);
        esp_zb_lock_release();
        LOG("[ZIGBEE] openNetwork(%d) called, waiting for join attempts...", timeout_sec);
    }

    bool join_window_open() {
        return permit_join_end_time > 0 && millis() < permit_join_end_time;
    }
    static bool zcl_raw_seen(uint8_t bufid);   // defined below, registered in init()

    // The coordinator is always short address 0x0000; our light is endpoint 3.
    #define ZB_COORDINATOR_SHORT_ADDR 0x0000
    #define ZB_LIGHT_ENDPOINT 3
    // the channel the coordinator forms its network on
    #define ZB_NETWORK_CHANNEL 20
    // esp_zb_bdb_open_network() takes a uint8_t number of seconds
    #define ZB_PERMIT_JOIN_MAX_SEC 254

    // State of our own light endpoint, shown on the status LED
    bool light_on = false;
    // Relay: our light is the one the remote controls; each change is forwarded
    // to relay_target (AT+ZBRELAY=). Sent from loop(), like the bind requests.
    bool relay_set = false;
    uint16_t relay_target = 0;
    bool relay_pending = false;
    bool relay_on = false;
    uint8_t relay_level = 255;
    uint8_t light_level = 255;   // 0-254 from the Level Control cluster, last value seen

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
        zbLight.setLightState(!zbLight.getLightState());
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
        zbLight.onLightChange([](bool on, uint8_t level) {
            // Someone on the network switched or dimmed our light: show it on the status LED
            light_on = on;
            light_level = level;
            if(relay_set) {
                relay_pending = true;
                relay_on = on;
                relay_level = level;
            }
            LOG("[ZIGBEE] Light switched %s, level %d", on ? "ON" : "OFF", level);
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
        // One channel only, on both boards: the coordinator forms the network on it,
        // and the end device scans only it. 20 is a primary channel (11, 15, 20, 25), the set
        // Hue bulbs look at first; 13 is not in it.
        Zigbee.setPrimaryChannelMask(1UL << ZB_NETWORK_CHANNEL);
#if defined(ZIGBEE_MODE_ZCZR)
        // Install-code policy: a device can join with the key from its install code (AT+ZBIC=)
        esp_zb_cfg_t zb_cfg = ZIGBEE_DEFAULT_COORDINATOR_CONFIG();
        zb_cfg.install_code_policy = true;
        bool zb_started = Zigbee.begin(&zb_cfg, false);
#else
        bool zb_started = Zigbee.begin(role);
#endif
        if(!zb_started) {
            LOG("[ZIGBEE] Failed to initialize Zigbee stack");
            return;
        }
        LOG("[ZIGBEE] Zigbee stack initialized successfully");
#if !defined(ZIGBEE_MODE_ED)
        esp_zb_raw_command_handler_register(zcl_raw_seen);
#endif
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

    // ---- Probing and binding a remote -----------------------------------
    // Ask a device for its endpoints and clusters (AT+ZBEPS=), then bind one
    // of its clients to our light (AT+ZBBIND=). Requests are sent from loop(),
    // never from a callback, so the Zigbee lock is taken once at a time.
    static uint16_t probe_addr = 0;
    static uint8_t probe_eps[8];
    static uint8_t probe_ep_count = 0;
    static uint8_t probe_next = 0;
    static bool probe_send_ep = false;     // active endpoint request due
    static bool probe_send_sd = false;     // simple descriptor request due
    static bool probe_done = false;
    static char probe_report[600];
    static size_t probe_len = 0;

    static void probe_append(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
    static void probe_append(const char *fmt, ...) {
        if(probe_len >= sizeof(probe_report)) return;
        va_list ap;
        va_start(ap, fmt);
        int n = vsnprintf(probe_report + probe_len, sizeof(probe_report) - probe_len, fmt, ap);
        va_end(ap);
        if(n > 0) probe_len += (size_t)n;
        if(probe_len >= sizeof(probe_report)) probe_len = sizeof(probe_report) - 1;
    }

    static void simple_desc_cb(esp_zb_zdp_status_t st, esp_zb_af_simple_desc_1_1_t *sd, void *ctx) {
        if(st == ESP_ZB_ZDP_STATUS_SUCCESS && sd) {
            probe_append("+ZBEPS:EP=%d,Profile=0x%04X,Device=0x%04X,In=",
                sd->endpoint, sd->app_profile_id, sd->app_device_id);
            for(int i = 0; i < sd->app_input_cluster_count; i++)
                probe_append("%04X ", sd->app_cluster_list[i]);
            probe_append(" Out=");
            for(int i = 0; i < sd->app_output_cluster_count; i++)
                probe_append("%04X ", sd->app_cluster_list[sd->app_input_cluster_count + i]);
            probe_append("\r\n");
        } else {
            probe_append("+ZBEPS:EP=%d,simple descriptor failed (status %d)\r\n", probe_eps[probe_next], st);
        }
        probe_next++;
        if(probe_next < probe_ep_count) {
            probe_send_sd = true;
        } else {
            probe_done = true;
            probe_append("+ZBEPS:done\r\n");
        }
    }

    static void active_ep_cb(esp_zb_zdp_status_t st, uint8_t count, uint8_t *list, void *ctx) {
        probe_len = 0;
        if(st != ESP_ZB_ZDP_STATUS_SUCCESS) {
            probe_append("+ZBEPS:active endpoint request failed (status %d)\r\n", st);
            probe_done = true;
            return;
        }
        probe_ep_count = count > sizeof(probe_eps) ? sizeof(probe_eps) : count;
        memcpy(probe_eps, list, probe_ep_count);
        probe_next = 0;
        probe_append("+ZBEPS:Device 0x%04X has %d endpoints\r\n", probe_addr, probe_ep_count);
        if(probe_ep_count == 0) {
            probe_done = true;
        } else {
            probe_send_sd = true;
        }
    }

    // A bind request to a sleeping device times out after 5 s, because the device
    // only listens for a moment after a button press. So keep resending it, every
    // BIND_RETRY_MS, until the device answers OK or BIND_GIVE_UP_MS has passed.
    #define BIND_RETRY_MS    6000
    #define BIND_GIVE_UP_MS  120000
    static bool bind_pending = false;
    static bool bind_send_now = false;
    static esp_zb_zdo_bind_req_param_t bind_req;
    static unsigned long bind_next_send = 0;
    static unsigned long bind_started = 0;
    static uint16_t bind_target = 0;
    static uint16_t bind_dst = 0;     // the device whose light gets bound (0x0000: ours)
    static bool bind_unbind = false;  // true: AT+ZBUNBIND, removes the binding instead

    // Every ZCL frame the stack receives passes here first. We only log that a
    // frame arrived; returning false lets the stack handle it as usual.
    static uint32_t zcl_frames_seen = 0;
    static bool zcl_raw_seen(uint8_t bufid) {
        zcl_frames_seen++;
        LOG("[ZIGBEE] ZCL frame received (#%u, buffer %u)", zcl_frames_seen, bufid);
        return false;
    }

    static void bind_cb(esp_zb_zdp_status_t st, void *ctx) {
        if(st == ESP_ZB_ZDP_STATUS_SUCCESS) {
            if(bind_unbind) {
                LOG("[ZIGBEE] Unbind OK: 0x%04X is no longer bound to light endpoint %d of 0x%04X", bind_target, ZB_LIGHT_ENDPOINT, bind_dst);
            } else {
                LOG("[ZIGBEE] Bind OK: 0x%04X is bound to light endpoint %d of 0x%04X", bind_target, ZB_LIGHT_ENDPOINT, bind_dst);
            }
            bind_pending = false;
        } else {
            LOG("[ZIGBEE] Bind to 0x%04X not answered (status %d), retrying", bind_target, st);
        }
    }

    // Sent from loop(): the requests need the Zigbee lock
    static void probe_service() {
        if(bind_pending) {
            unsigned long now = millis();
            if(now - bind_started > BIND_GIVE_UP_MS) {
                LOG("[ZIGBEE] Giving up on binding 0x%04X after %d s", bind_target, BIND_GIVE_UP_MS / 1000);
                bind_pending = false;
            } else if(bind_send_now || now >= bind_next_send) {
                bind_send_now = false;
                bind_next_send = now + BIND_RETRY_MS;
                if(esp_zb_lock_acquire(pdMS_TO_TICKS(1000))) {
                    if(bind_unbind) esp_zb_zdo_device_unbind_req(&bind_req, bind_cb, NULL);
                    else esp_zb_zdo_device_bind_req(&bind_req, bind_cb, NULL);
                    esp_zb_lock_release();
                }
            }
        }
        if(probe_send_ep) {
            probe_send_ep = false;
            esp_zb_zdo_active_ep_req_param_t req;
            memset(&req, 0, sizeof(req));
            req.addr_of_interest = probe_addr;
            if(esp_zb_lock_acquire(pdMS_TO_TICKS(1000))) {
                esp_zb_zdo_active_ep_req(&req, active_ep_cb, NULL);
                esp_zb_lock_release();
            } else {
                probe_append("+ZBEPS:could not take the Zigbee lock\r\n");
                probe_done = true;
            }
        }
        if(probe_send_sd) {
            probe_send_sd = false;
            esp_zb_zdo_simple_desc_req_param_t req;
            memset(&req, 0, sizeof(req));
            req.addr_of_interest = probe_addr;
            req.endpoint = probe_eps[probe_next];
            if(esp_zb_lock_acquire(pdMS_TO_TICKS(1000))) {
                esp_zb_zdo_simple_desc_req(&req, simple_desc_cb, NULL);
                esp_zb_lock_release();
            } else {
                probe_append("+ZBEPS:could not take the Zigbee lock\r\n");
                probe_done = true;
            }
        }
    }

    // ---- Thermostat valve (read and write its thermostat cluster) -------
    // The valve sleeps like the STYRBAR, so a request is resent every
    // BIND_RETRY_MS until the valve answers, or BIND_GIVE_UP_MS has passed.
    enum valve_op_t { VALVE_NONE, VALVE_READ, VALVE_WRITE };
    static valve_op_t valve_op = VALVE_NONE;
    static uint16_t valve_addr = 0;
    static uint8_t valve_ep = 1;
    static int16_t valve_write_value = 0;   // 0.01 deg C units
    static bool valve_send_now = false;
    static unsigned long valve_next_send = 0;
    static unsigned long valve_started = 0;

    static void valve_send() {
        if(!esp_zb_lock_acquire(pdMS_TO_TICKS(1000)))
            return;
        if(valve_op == VALVE_READ) {
            static uint16_t attrs[2] = { ESP_ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
                                         ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID };
            esp_zb_zcl_read_attr_cmd_t cmd;
            memset(&cmd, 0, sizeof(cmd));
            cmd.zcl_basic_cmd.src_endpoint = zbThermostat.getEndpoint();
            cmd.zcl_basic_cmd.dst_endpoint = valve_ep;
            cmd.zcl_basic_cmd.dst_addr_u.addr_short = valve_addr;
            cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
            cmd.attr_number = 2;
            cmd.attr_field = attrs;
            esp_zb_zcl_read_attr_cmd_req(&cmd);
        } else if(valve_op == VALVE_WRITE) {
            esp_zb_zcl_attribute_t attr;
            memset(&attr, 0, sizeof(attr));
            attr.id = ESP_ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID;
            attr.data.type = ESP_ZB_ZCL_ATTR_TYPE_S16;
            attr.data.size = sizeof(valve_write_value);
            attr.data.value = &valve_write_value;
            esp_zb_zcl_write_attr_cmd_t cmd;
            memset(&cmd, 0, sizeof(cmd));
            cmd.zcl_basic_cmd.src_endpoint = zbThermostat.getEndpoint();
            cmd.zcl_basic_cmd.dst_endpoint = valve_ep;
            cmd.zcl_basic_cmd.dst_addr_u.addr_short = valve_addr;
            cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_THERMOSTAT;
            cmd.attr_number = 1;
            cmd.attr_field = &attr;
            esp_zb_zcl_write_attr_cmd_req(&cmd);
        }
        esp_zb_lock_release();
    }

    static void valve_service() {
        if(valve_op == VALVE_NONE)
            return;
        unsigned long now = millis();
        if(zbThermostat.answered) {
            valve_op = VALVE_NONE;
            return;
        }
        if(now - valve_started > BIND_GIVE_UP_MS) {
            LOG("[VALVE] No answer from 0x%04X after %d s, giving up", valve_addr, BIND_GIVE_UP_MS / 1000);
            valve_op = VALVE_NONE;
            return;
        }
        if(valve_send_now || now >= valve_next_send) {
            valve_send_now = false;
            valve_next_send = now + BIND_RETRY_MS;
            valve_send();
        }
    }

    // Forward the light's last on/off and level to the relay target
    static void relay_service() {
        if(!relay_pending) return;
        relay_pending = false;
        if(!esp_zb_lock_acquire(pdMS_TO_TICKS(1000))) {
            LOG("[ZIGBEE] Relay: could not take the Zigbee lock, dropped");
            return;
        }
        esp_zb_zcl_on_off_cmd_t on_cmd;
        memset(&on_cmd, 0, sizeof(on_cmd));
        on_cmd.zcl_basic_cmd.src_endpoint = zbLight.getEndpoint();
        on_cmd.zcl_basic_cmd.dst_endpoint = ZB_LIGHT_ENDPOINT;
        on_cmd.zcl_basic_cmd.dst_addr_u.addr_short = relay_target;
        on_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
        on_cmd.on_off_cmd_id = relay_on ? ESP_ZB_ZCL_CMD_ON_OFF_ON_ID : ESP_ZB_ZCL_CMD_ON_OFF_OFF_ID;
        esp_zb_zcl_on_off_cmd_req(&on_cmd);
        if(relay_on) {
            esp_zb_zcl_move_to_level_cmd_t lvl;
            memset(&lvl, 0, sizeof(lvl));
            lvl.zcl_basic_cmd.src_endpoint = zbLight.getEndpoint();
            lvl.zcl_basic_cmd.dst_endpoint = ZB_LIGHT_ENDPOINT;
            lvl.zcl_basic_cmd.dst_addr_u.addr_short = relay_target;
            lvl.address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT;
            lvl.level = relay_level;
            lvl.transition_time = 0;
            esp_zb_zcl_level_move_to_level_cmd_req(&lvl);
        }
        esp_zb_lock_release();
        LOG("[ZIGBEE] Relayed to 0x%04X: %s, level %d", relay_target, relay_on ? "ON" : "OFF", relay_level);
    }

    void loop() {
        probe_service();
        relay_service();
        valve_service();
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
            // A new child in the neighbour table: the LED goes steady (checked twice a second)
            static unsigned long last_join_check = 0;
            if (!join_device_joined && now - last_join_check >= 500) {
                last_join_check = now;
                size_t n = count_children();
                if (n > join_children_at_open) {
                    join_device_joined = true;
                    LOG("[ZIGBEE] *** A device joined *** %d devices on the network", n);
                    log_children();
                }
            }
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

    // Print each child with its IEEE and short address
    void log_children() {
        Neighbor nbrs[16];
        size_t n = get_neighbors(nbrs, 16);
        for(size_t i = 0; i < n; i++) {
            if(nbrs[i].relationship != ESP_ZB_NWK_RELATIONSHIP_CHILD
               && nbrs[i].relationship != ESP_ZB_NWK_RELATIONSHIP_UNAUTHENTICATED_CHILD)
                continue;
            LOG("[ZIGBEE]   child IEEE=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X Short=0x%04X",
                nbrs[i].ieee_addr[7], nbrs[i].ieee_addr[6], nbrs[i].ieee_addr[5], nbrs[i].ieee_addr[4],
                nbrs[i].ieee_addr[3], nbrs[i].ieee_addr[2], nbrs[i].ieee_addr[1], nbrs[i].ieee_addr[0],
                nbrs[i].short_addr);
        }
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
        // AT+ZBINFO? - Our own network: PAN ID, channel, address, permit-join state
        else if (p = COMMON::at_cmd_check("AT+ZBINFO?", at_cmd, cmd_len)) {
            if (!Zigbee.started()) {
                return AT_R("+ZBINFO:stack not started");
            }
            esp_zb_ieee_addr_t ext_pan = {0};
            esp_zb_get_extended_pan_id(ext_pan);
            long permit_left = 0;
            if (permit_join_end_time > 0 && millis() < permit_join_end_time) {
                permit_left = (permit_join_end_time - millis()) / 1000;
            }
            snprintf(response, sizeof(response),
                "+ZBINFO:Role=%s,Connected=%s,PANID=0x%04X,ExtPANID=%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X,"
                "Channel=%d,Short=0x%04X,PermitJoin=%lds\r\n",
                role == ZIGBEE_COORDINATOR ? "coordinator" : (role == ZIGBEE_ROUTER ? "router" : "end-device"),
                Zigbee.connected() ? "yes" : "no",
                esp_zb_get_pan_id(),
                ext_pan[7], ext_pan[6], ext_pan[5], ext_pan[4], ext_pan[3], ext_pan[2], ext_pan[1], ext_pan[0],
                esp_zb_get_current_channel(), esp_zb_get_short_address(), permit_left);
            return response;
        }
        // AT+ZBEPS=<short> - Ask a device for its endpoints and clusters; read with AT+ZBEPS?
        else if (p = COMMON::at_cmd_check("AT+ZBEPS=", at_cmd, cmd_len)) {
            unsigned int addr = 0;
            if (sscanf(p, "%x", &addr) != 1) {
                return AT_R("+ERROR: Use AT+ZBEPS=<short address in hex, e.g. 74C5>");
            }
            probe_addr = (uint16_t)addr;
            probe_len = 0;
            probe_done = false;
            probe_send_ep = true;
            snprintf(response, sizeof(response), "+ZBEPS:probing 0x%04X, read it with AT+ZBEPS?\r\nOK", probe_addr);
            return response;
        }
        else if (p = COMMON::at_cmd_check("AT+ZBEPS?", at_cmd, cmd_len)) {
            if (!probe_done) {
                return AT_R("+ZBEPS:not ready yet, try again in a second");
            }
            probe_report[probe_len] = 0;
            return probe_report;
        }
        // AT+ZBIC=<ieee>,<install code> - Register a device's install code, so it can join with
        // the key derived from it. <ieee> is the 16 hex digits printed on the device (MSB first),
        // <install code> the 36 hex digits from the label (16 bytes + 2 bytes CRC). The install
        // code is never logged, and the call must come before the device joins.
        else if (p = COMMON::at_cmd_check("AT+ZBIC=", at_cmd, cmd_len)) {
            size_t n = strcspn(p, "\r\n");
            if (n != 16 + 1 + 36 || p[16] != ',') {
                return AT_R("+ERROR: Use AT+ZBIC=<16 hex ieee>,<36 hex install code>");
            }
            esp_zb_ieee_addr_t ieee;
            for (int i = 0; i < 8; i++) {
                char byte[3] = { p[2 * i], p[2 * i + 1], 0 };
                char *end = NULL;
                unsigned long v = strtoul(byte, &end, 16);
                if (end == byte || *end != 0) {
                    return AT_R("+ERROR: The ieee address is not hex");
                }
                ieee[7 - i] = (uint8_t)v;   // the stack keeps the address least significant byte first
            }
            char ic[37];
            memcpy(ic, p + 17, 36);
            ic[36] = 0;
            for (int i = 0; i < 36; i++) {
                if (!isxdigit((unsigned char)ic[i])) {
                    memset(ic, 0, sizeof(ic));
                    return AT_R("+ERROR: The install code is not hex");
                }
            }
            if (!esp_zb_lock_acquire(pdMS_TO_TICKS(1000))) {
                memset(ic, 0, sizeof(ic));
                return AT_R("+ERROR: Could not take the Zigbee lock");
            }
#if defined(ZIGBEE_MODE_ZCZR)
            esp_err_t err = esp_zb_secur_ic_str_add(ieee, ic);
            esp_zb_lock_release();
#else
            // only the coordinator is a trust center, so only it can hold install codes
            esp_err_t err = ESP_ERR_NOT_SUPPORTED;
            esp_zb_lock_release();
#endif
            memset(ic, 0, sizeof(ic));
            if (err != ESP_OK) {
                LOG("[ZIGBEE] Install code for %.16s rejected (error %d)", p, (int)err);
                return AT_R("+ERROR: The install code was rejected");
            }
            LOG("[ZIGBEE] Install code added for device %.16s", p);
            return AT_R("OK");
        }
        // AT+ZBRELAY=<short> - Forward our light's on/off and level to <short> (0 to stop)
        else if (p = COMMON::at_cmd_check("AT+ZBRELAY=", at_cmd, cmd_len)) {
            unsigned int addr = 0;
            if (sscanf(p, "%x", &addr) != 1) {
                return AT_R("+ERROR: Use AT+ZBRELAY=<short hex>, or AT+ZBRELAY=0 to stop");
            }
            relay_set = (addr != 0);
            relay_target = (uint16_t)addr;
            LOG("[ZIGBEE] Relay %s, target 0x%04X", relay_set ? "on" : "off", (uint16_t)addr);
            return AT_R("OK");
        }
        // AT+ZBBIND=<short>,<src_ep>[,<dst_short>[,<cluster>]] - Bind the remote's client
        // cluster on src_ep (default On/Off) to a light endpoint: ours, or the light of <dst_short>.
        // AT+ZBUNBIND= takes the same arguments and removes the binding.
        else if ((p = COMMON::at_cmd_check("AT+ZBBIND=", at_cmd, cmd_len)) || (p = COMMON::at_cmd_check("AT+ZBUNBIND=", at_cmd, cmd_len))) {
            bind_unbind = (strncmp(at_cmd, "AT+ZBUNBIND", 11) == 0);
            unsigned int addr = 0, src_ep = 0, dst = ZB_COORDINATOR_SHORT_ADDR;
            unsigned int cluster = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF;
            int got = sscanf(p, "%x,%u,%x,%x", &addr, &src_ep, &dst, &cluster);
            if (got < 2) {
                return AT_R("+ERROR: Use AT+ZBBIND=<short hex>,<endpoint>[,<target short hex>], e.g. AT+ZBBIND=74C5,1,181F");
            }
            // the IEEE addresses come from our neighbour table
            Neighbor nbrs[16];
            size_t n = get_neighbors(nbrs, 16);
            bool found = false, found_dst = false;
            esp_zb_zdo_bind_req_param_t req;
            memset(&req, 0, sizeof(req));
            for (size_t i = 0; i < n; i++) {
                if (nbrs[i].short_addr == (uint16_t)addr) {
                    memcpy(req.src_address, nbrs[i].ieee_addr, 8);
                    found = true;
                }
                if (got >= 3 && dst != ZB_COORDINATOR_SHORT_ADDR && nbrs[i].short_addr == (uint16_t)dst) {
                    memcpy(req.dst_address_u.addr_long, nbrs[i].ieee_addr, 8);
                    found_dst = true;
                }
            }
            if (!found) {
                return AT_R("+ERROR: That device is not on our network (see AT+ZBNEIGHBORS?)");
            }
            if (got >= 3 && dst != ZB_COORDINATOR_SHORT_ADDR && !found_dst) {
                return AT_R("+ERROR: The target device is not on our network (see AT+ZBNEIGHBORS?)");
            }
            if (got < 3 || dst == ZB_COORDINATOR_SHORT_ADDR) {
                esp_zb_get_long_address(req.dst_address_u.addr_long);
                dst = ZB_COORDINATOR_SHORT_ADDR;
            }
            req.src_endp = (uint8_t)src_ep;
            req.cluster_id = (uint16_t)cluster;
            req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
            req.dst_endp = ZB_LIGHT_ENDPOINT;
            req.req_dst_addr = (uint16_t)addr;
            // loop() sends it now and keeps resending until the device answers
            bind_req = req;
            bind_target = (uint16_t)addr;
            bind_dst = (uint16_t)dst;
            bind_pending = true;
            bind_send_now = true;
            bind_started = millis();
            snprintf(response, sizeof(response),
                "+ZBBIND:retrying 0x%04X endpoint %u -> our light endpoint %d for up to %d s, watch the log for 'Bind OK'\r\nOK",
                (uint16_t)addr, src_ep, ZB_LIGHT_ENDPOINT, BIND_GIVE_UP_MS / 1000);
            return response;
        }
        // AT+ZBREAD=<short>,<ep> - Read the valve's room temperature and setpoint
        else if (p = COMMON::at_cmd_check("AT+ZBREAD=", at_cmd, cmd_len)) {
            unsigned int addr = 0, ep = 0;
            if (sscanf(p, "%x,%u", &addr, &ep) != 2 || ep == 0 || ep > 240) {
                return AT_R("+ERROR: Use AT+ZBREAD=<short hex>,<endpoint>, e.g. AT+ZBREAD=74C5,1");
            }
            valve_op = VALVE_READ;
            valve_addr = (uint16_t)addr;
            valve_ep = (uint8_t)ep;
            valve_send_now = true;
            valve_started = millis();
            zbThermostat.answered = false;
            snprintf(response, sizeof(response), "+ZBREAD:requested from 0x%04X endpoint %u, read it with AT+ZBVALVE?\r\nOK",
                     valve_addr, valve_ep);
            return response;
        }
        // AT+ZBSETPOINT=<short>,<ep>,<C> - Write the valve's heating setpoint
        else if (p = COMMON::at_cmd_check("AT+ZBSETPOINT=", at_cmd, cmd_len)) {
            unsigned int addr = 0, ep = 0;
            float c = 0;
            if (sscanf(p, "%x,%u,%f", &addr, &ep, &c) != 3 || ep == 0 || ep > 240) {
                return AT_R("+ERROR: Use AT+ZBSETPOINT=<short hex>,<endpoint>,<deg C>, e.g. AT+ZBSETPOINT=74C5,1,21");
            }
            if (c < 5.0f || c > 30.0f) {
                return AT_R("+ERROR: Setpoint must be between 5 and 30 deg C");
            }
            valve_op = VALVE_WRITE;
            valve_addr = (uint16_t)addr;
            valve_ep = (uint8_t)ep;
            valve_write_value = (int16_t)lroundf(c * 100.0f);
            valve_send_now = true;
            valve_started = millis();
            zbThermostat.answered = false;
            snprintf(response, sizeof(response), "+ZBSETPOINT:sending %.2f C to 0x%04X endpoint %u, watch the log for the write status\r\nOK",
                     c, valve_addr, valve_ep);
            return response;
        }
        // AT+ZBVALVE? - Last values read from the valve
        else if (p = COMMON::at_cmd_check("AT+ZBVALVE?", at_cmd, cmd_len)) {
            snprintf(response, sizeof(response), "+ZBVALVE:room=%s,setpoint=%s,answered=%s",
                     isnan(zbThermostat.local_temp) ? "unknown" : String(zbThermostat.local_temp, 2).c_str(),
                     isnan(zbThermostat.setpoint) ? "unknown" : String(zbThermostat.setpoint, 2).c_str(),
                     zbThermostat.answered ? "yes" : "no");
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
            zbLight.setLightState(*p == '1');
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
                    // openNetwork() takes a uint8_t: anything over 255 wraps, so
                    // AT+ZBJOIN=280 would really open the network for 24 seconds
                    // while we reported 280. Clamp instead, and report the truth.
                    if (timeout_sec > ZB_PERMIT_JOIN_MAX_SEC) {
                        LOG("[ZIGBEE] Permit join time %d capped to %d seconds", timeout_sec, ZB_PERMIT_JOIN_MAX_SEC);
                        timeout_sec = ZB_PERMIT_JOIN_MAX_SEC;
                    }
                    open_join_window(timeout_sec);
                    snprintf(response, sizeof(response), "+ZBJOIN:permit_join_enabled,%d\r\nOK", timeout_sec);
                } else {
                    LOG("[ZIGBEE] Disabling permit joining");
                    permit_join_end_time = 0;
                    if(esp_zb_lock_acquire(pdMS_TO_TICKS(1000))) {
                        Zigbee.closeNetwork();
                        esp_zb_lock_release();
                    }
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
  AT+ZBINFO?          - Our network: PAN ID, channel, address, permit-join left
  AT+ZBEPS=<short>    - Ask a device for its endpoints and clusters (read with AT+ZBEPS?)
  AT+ZBBIND=<short>,<ep>[,<target>[,<cluster>]] - Bind the device's client cluster
                        on <ep> (default On/Off, 0006) to the light of <target>
                        (default: our own light)
  AT+ZBUNBIND=...     - Same arguments as AT+ZBBIND, removes that binding
  AT+ZBRELAY=<short>  - Forward our light's on/off and level to <short> (0 stops)
  AT+ZBREAD=<short>,<ep> - Read a thermostat valve's room temperature and setpoint
  AT+ZBSETPOINT=<short>,<ep>,<C> - Set a thermostat valve's heating setpoint (5-30 C)
  AT+ZBVALVE?         - Last values read from the valve
  AT+ZBTOGGLE         - Toggle a light: on an end device the coordinator's (as
                        a 2-5 s button press there does), else our own
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

extern uint8_t led_rgb_scale;   // defined in esp-at.ino, at file scope

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
            // dimmed light: the LED gets dimmer, from 25% at the lowest level to full
            led_rgb_scale = 64 + (191 * ZIGBEE::light_level) / 254;
            // orange, not green: against the dim blue idle glow green is hard to tell apart
            return LED_RGB_ORANGE;
        }
        led_rgb_scale = 255;   // light off: full brightness again
#if defined(ZIGBEE_MODE_ED)
        // The end device is the board with the button: purple, so the two
        // boards are easy to tell apart. The coordinator keeps the dim blue.
        return LED_RGB_IDLE_PURPLE;
#else
        return -1;
#endif
    }
    void button_long_press() {
        // Toggle the light on the coordinator (on the coordinator itself, our own)
        ZIGBEE::button_toggle_light();
    }
    int led_status(int *interval_ms) {
        // Join mode: magenta, blinking while we wait, steady once a device joined
        if(!ZIGBEE::join_window_open())
            return -1;
        *interval_ms = ZIGBEE::join_device_joined ? 0 : 250;
        return LED_RGB_MAGENTA;
    }
#if !defined(ZIGBEE_MODE_ED)
    void button_hold_5s() {
        // BOOT held for 5 s on the coordinator: open the network for joining
        LOG("[ZIGBEE] Button held 5 s: join mode");
        ZIGBEE::open_join_window(ZB_PERMIT_JOIN_MAX_SEC);
    }
#endif
}
