/*
  AP_ExternalAHRS JSBSim backend implementation.
 */
#include "AP_ExternalAHRS_JSBSim.h"

#if AP_EXTERNAL_AHRS_JSBSIM_ENABLED

#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_AllSpark/AP_AllSpark.h>

extern const AP_HAL::HAL& hal;

AP_ExternalAHRS_JSBSim::AP_ExternalAHRS_JSBSim(AP_ExternalAHRS *_frontend, AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    AP_SerialManager &sm = AP::serialmanager();
    _port = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (_port != nullptr) {
        _port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
        _port->begin(sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0));
        // this backend owns the port: BRD_SERn_RTSCTS may have enabled hardware
        // flow control before begin(), and a 3-wire link leaves CTS floating
        // high, which would block RX entirely.
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }
    _rx_ofs = 0;
    _healthy = false;
    _initialised = false;

}

uint16_t AP_ExternalAHRS_JSBSim::crc16_init0(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

void AP_ExternalAHRS_JSBSim::update(void)
{
    if (_port == nullptr) {
        return;
    }
    while (_port->available() > 0) {
        uint8_t b;
        if (!_port->read(b)) {
            break;
        }
        process_byte(b);
    }
}

void AP_ExternalAHRS_JSBSim::process_byte(uint8_t b)
{
    if (_rx_ofs >= sizeof(_rx_buf)) {
        memmove(_rx_buf, &_rx_buf[1], sizeof(_rx_buf) - 1);
        _rx_ofs--;
    }
    _rx_buf[_rx_ofs++] = b;

    // align to a recognized magic: JBS2 (sensor) or JBA1 (fault receipt)
    while (_rx_ofs >= 4 && memcmp(_rx_buf, "JBS2", 4) != 0 && memcmp(_rx_buf, "JBA1", 4) != 0) {
        memmove(_rx_buf, &_rx_buf[1], _rx_ofs - 1);
        _rx_ofs--;
    }
    if (_rx_ofs < 6) {
        return;
    }
    const bool is_jbs2 = (memcmp(_rx_buf, "JBS2", 4) == 0);
    const uint16_t plen = _rx_buf[4] | (_rx_buf[5] << 8);
    const uint16_t min_len = is_jbs2 ? (uint16_t)sizeof(JSBSimSensorPayload) : 9;
    if (plen < min_len || plen > sizeof(_rx_buf) - 8) {
        memmove(_rx_buf, &_rx_buf[1], _rx_ofs - 1);
        _rx_ofs--;
        return;
    }
    if (_rx_ofs < 6 + plen + 2) {
        return;
    }
    const uint16_t crc_calc = crc16_init0(_rx_buf, 6 + plen);
    const uint16_t crc_rx = _rx_buf[6 + plen] | (_rx_buf[7 + plen] << 8);
    if (crc_calc == crc_rx) {
        if (is_jbs2) {
            handle_payload(&_rx_buf[6], plen);
        } else {
#if AP_ALLSPARK_ENABLED
            // JBA1 fault receipt: hand to the AllSpark fault transaction
            if (AP::allspark() != nullptr) {
                AP::allspark()->handle_jba1_payload(&_rx_buf[6], plen);
            }
#endif
        }
    }
    _rx_ofs = 0;
}

void AP_ExternalAHRS_JSBSim::handle_payload(const uint8_t *payload, uint16_t len)
{
    if (len < sizeof(JSBSimSensorPayload)) {
        return;
    }
    JSBSimSensorPayload p;
    memcpy(&p, payload, sizeof(p));

    {
        WITH_SEMAPHORE(state.sem);

        state.quat.from_euler(p.roll, p.pitch, p.yaw);
        state.have_quaternion = true;

        state.gyro = Vector3f(p.p, p.q, p.r);
        state.accel = Vector3f(p.ax, p.ay, p.az);

        state.location.lat = p.latitude;
        state.location.lng = p.longitude;
        state.location.set_alt_cm(p.altitude, Location::AltFrame::ABSOLUTE);
        state.have_location = true;
        state.last_location_update_us = AP_HAL::micros();

        if (!state.have_origin && p.fix_type >= uint8_t(AP_GPS_FixType::FIX_3D)) {
            state.origin = state.location;
            state.have_origin = true;
        }

        state.velocity = Vector3f(p.vn, p.ve, p.vd);
        state.have_velocity = true;
    }

    _healthy = true;
    _initialised = true;
    _last_update_ms = AP_HAL::millis();
    _gps_fix_type = p.fix_type;

    // feed INS (gyro + accel) so the external IMU is used as the primary IMU
    {
        const AP_ExternalAHRS::ins_data_message_t ins {
            accel: Vector3f(p.ax, p.ay, p.az),
            gyro: Vector3f(p.p, p.q, p.r),
            temperature: p.temperature,
        };
        AP::ins().handle_external(ins);
    }

    // feed baro (static pressure + temperature)
#if AP_BARO_EXTERNALAHRS_ENABLED
    {
        const AP_ExternalAHRS::baro_data_message_t baro {
            instance: 0,
            pressure_pa: p.static_pressure,
            temperature: p.temperature,
        };
        AP::baro().handle_external(baro);
    }
#endif

    // IMU frames repeat the GPS time between GNSS samples. Do not publish
    // those repeats as new fixes or keep a frozen GPS alive with IMU traffic.
    if (!_have_gps_time || p.gps_week != _last_gps_week || p.gps_week_ms != _last_gps_week_ms) {
        const AP_ExternalAHRS::gps_data_message_t gps {
            gps_week: p.gps_week,
            ms_tow: p.gps_week_ms,
            fix_type: AP_GPS_FixType(p.fix_type),
            satellites_in_view: p.num_sats,
            horizontal_pos_accuracy: p.h_acc,
            vertical_pos_accuracy: p.v_acc,
            horizontal_vel_accuracy: p.hv_acc,
            hdop: p.hdop,
            vdop: p.vdop,
            longitude: p.longitude,
            latitude: p.latitude,
            msl_altitude: p.altitude,
            ned_vel_north: p.vn,
            ned_vel_east: p.ve,
            ned_vel_down: p.vd,
        };
        uint8_t gps_instance;
        if (AP::gps().get_first_external_instance(gps_instance)) {
            AP::gps().handle_external(gps, gps_instance);
            _have_gps_time = true;
            _last_gps_week = p.gps_week;
            _last_gps_week_ms = p.gps_week_ms;
            _last_gps_update_ms = _last_update_ms;
        }
    }

    // feed airspeed (dynamic pressure) for ARSPD_TYPE = 16 (external).
    // Only plane and copter link the full AP_Airspeed class.
#if AP_AIRSPEED_EXTERNAL_ENABLED && (APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane))
    {
        const AP_ExternalAHRS::airspeed_data_message_t airspeed {
            differential_pressure: p.dynamic_pressure,
            temperature: p.temperature,
        };
        AP::airspeed().handle_external(airspeed);
    }
#endif

    // parse the 36-byte truth extension if present
    if (len >= sizeof(JSBSimSensorPayload) + sizeof(JSBSimTruthPayload)) {
        JSBSimTruthPayload t;
        memcpy(&t, payload + sizeof(JSBSimSensorPayload), sizeof(t));

        AP_ExternalAHRS::JSBSimTruth truth {};
        truth.valid = true;
        truth.valid_mask = t.valid_mask;
        truth.aoa_deg = t.aoa_cd * 0.01f;
        truth.sideslip_deg = t.sideslip_cd * 0.01f;
        memcpy(truth.actual_surface_cd, t.actual_surface_cd, sizeof(truth.actual_surface_cd));
        memcpy(truth.actual_throttle_permille, t.actual_throttle_permille, sizeof(truth.actual_throttle_permille));
        truth.active_fault_count = t.active_fault_count;
        truth.active_fault_mask = (uint8_t)t.active_fault_mask;
        truth.fault_slot = t.fault_slot;
        truth.fault_type = t.fault_type;
        truth.fault_location = t.fault_location;
        truth.fault_state = t.fault_state;
        truth.severity_permille = t.severity_permille;
        truth.parameter_1_q10000 = t.parameter_1_q10000;
        truth.parameter_2_q10000 = t.parameter_2_q10000;
        set_jsbsim_truth(truth);
    }
}

bool AP_ExternalAHRS_JSBSim::healthy(void) const
{
    return _healthy && (AP_HAL::millis() - _last_update_ms < 500);
}

bool AP_ExternalAHRS_JSBSim::initialised(void) const
{
    return _initialised;
}

bool AP_ExternalAHRS_JSBSim::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "JSBSimAHRS unhealthy");
        return false;
    }
    return true;
}

void AP_ExternalAHRS_JSBSim::get_filter_status(nav_filter_status &status) const
{
    memset(&status, 0, sizeof(status));
    const bool data_healthy = healthy();
    // Navigation needs a fresh 3D fix, even while IMU frames keep arriving.
    // Plane uses using_gps when deciding whether to fall back to DCM after arming.
    const bool gps_healthy = data_healthy && _have_gps_time &&
        _gps_fix_type >= uint8_t(AP_GPS_FixType::FIX_3D) &&
        AP_HAL::millis() - _last_gps_update_ms < 500;
    status.flags.initalized = _initialised;
    status.flags.attitude = data_healthy;
    status.flags.vert_vel = data_healthy;
    status.flags.vert_pos = data_healthy;
    status.flags.horiz_vel = gps_healthy;
    status.flags.horiz_pos_rel = gps_healthy;
    status.flags.horiz_pos_abs = gps_healthy;
    status.flags.using_gps = gps_healthy;
}

bool AP_ExternalAHRS_JSBSim::get_variances(float &velVar, float &posVar, float &hgtVar, Vector3f &magVar, float &tasVar) const
{
    // JSBSim does not report variances
    return false;
}

#endif  // AP_EXTERNAL_AHRS_JSBSIM_ENABLED
