/*
  AP_ExternalAHRS JSBSim backend

  Receives attitude/velocity/position/baro/GPS truth from a JSBSim
  flight-dynamics model over the AHRS serial link (SERIALn_PROTOCOL = 36)
  using "JBS2" frames, and reports GPS/baro data to the rest of ArduPilot.

  JBS2 frame (shared wrapper, protocol doc section 11):
    magic[4] "JBS2" + payload_len u16(LE) + payload + CRC16 u16(LE)
    CRC poly 0x1021, init 0, over magic..payload.

  payload = JSBSimSensorPayload (116 bytes) + JSBSimTruthPayload (36 bytes).

  Sensor layout matches JSBSim/hil_supervisor.py JSBSIM_SENSOR_PAYLOAD:
    "<I f*12 iii f*6 BBHI f*5"
  Truth layout matches JSBSIM_TRUTH_PAYLOAD: "<BBHBBBBHiihh4h2HH".
 */
#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_JSBSIM_ENABLED

#include "AP_ExternalAHRS_backend.h"
#include <AP_HAL/AP_HAL.h>

// 116-byte sensor payload
struct PACKED JSBSimSensorPayload {
    uint32_t time_ms;
    float roll, pitch, yaw;         // rad
    float p, q, r;                  // rad/s body rates
    float ax, ay, az;               // m/s^2 body accel
    float mag_x, mag_y, mag_z;      // mag field (unused by JSBSim)
    int32_t latitude;               // deg * 1e7
    int32_t longitude;              // deg * 1e7
    int32_t altitude;               // cm MSL
    float vn, ve, vd;               // m/s NED
    float static_pressure;          // Pa
    float dynamic_pressure;         // Pa
    float temperature;              // deg C
    uint8_t fix_type;               // 3 = 3D fix
    uint8_t num_sats;
    uint16_t gps_week;
    uint32_t gps_week_ms;
    float h_acc;                    // horizontal pos accuracy m
    float v_acc;                    // vertical pos accuracy m
    float hv_acc;                   // horizontal vel accuracy m/s
    float hdop;
    float vdop;
};

// 36-byte truth extension (AoA / sideslip / real surfaces / fault truth)
struct PACKED JSBSimTruthPayload {
    uint8_t sysid;
    uint8_t active_fault_count;
    uint16_t active_fault_mask;
    uint8_t fault_slot;
    uint8_t fault_type;
    uint8_t fault_location;
    uint8_t fault_state;
    uint16_t severity_permille;
    int32_t parameter_1_q10000;
    int32_t parameter_2_q10000;
    int16_t aoa_cd;                 // deg * 100
    int16_t sideslip_cd;            // deg * 100
    int16_t actual_surface_cd[4];   // aileron, elevator, rudder, reserved
    uint16_t actual_throttle_permille[2];
    uint16_t valid_mask;
};

static_assert(sizeof(JSBSimSensorPayload) == 116, "JSBSim sensor payload must be 116 bytes");
static_assert(sizeof(JSBSimTruthPayload) == 36, "JSBSim truth payload must be 36 bytes");

class AP_ExternalAHRS_JSBSim : public AP_ExternalAHRS_backend {
public:
    AP_ExternalAHRS_JSBSim(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state);

    const char* get_name() const override { return "JSBSim"; }
    int8_t get_port(void) const override { return _port_num; }

    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    bool get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const override;

    void update() override;

private:
    AP_HAL::UARTDriver *_port;
    int8_t _port_num = -1;

    bool _healthy;
    bool _initialised;
    uint32_t _last_update_ms;
    bool _have_gps_time;
    uint16_t _last_gps_week;
    uint32_t _last_gps_week_ms;
    uint32_t _last_gps_update_ms;
    uint8_t _gps_fix_type;

    // rx parser
    uint8_t _rx_buf[8 + 200];
    uint16_t _rx_ofs;

    void process_byte(uint8_t b);
    void handle_payload(const uint8_t *payload, uint16_t len);
    static uint16_t crc16_init0(const uint8_t *buf, uint16_t len);

    uint8_t num_gps_sensors(void) const override { return 1; }
};

#endif  // AP_EXTERNAL_AHRS_JSBSIM_ENABLED
