/*
  AP_AllSpark implementation - see header for protocol description.
 */
#include "AP_AllSpark.h"

#if AP_ALLSPARK_ENABLED

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_FaultInjection/AP_FaultInjection.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

#include <string.h>
#include <math.h>

extern const AP_HAL::HAL& hal;

AP_AllSpark *AP_AllSpark::singleton = nullptr;

// table of user settable parameters
const AP_Param::GroupInfo AP_AllSpark::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: AllSpark protocol enable
    // @Description: Enable the AllSpark binary protocol on the serial port configured with SERIALn_PROTOCOL=51. This is the authorisation boundary for the link: with the protocol disabled the port is not opened and no guidance or fault command is accepted.
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO("ENABLE", 1, AP_AllSpark, enable, 1),

    // @Param: OPTIONS
    // @DisplayName: AllSpark protocol options
    // @Description: Options for the AllSpark binary protocol link.
    // @Bitmask: 0:AllowGuidanceWhenDisarmed
    // @User: Advanced
    AP_GROUPINFO("OPTIONS", 2, AP_AllSpark, options, 0),

    // @Param: TIMEOUT
    // @DisplayName: AllSpark guidance timeout
    // @Description: Time in seconds after which an AllSpark guidance command is considered stale and stops being applied. A guidance command carrying its own timeout overrides this value for the duration of that command. Zero disables the timeout. The flight mode is never changed by the timeout; only the AllSpark targets are dropped.
    // @Units: s
    // @Range: 0 60
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("TIMEOUT", 3, AP_AllSpark, guidance_timeout_s, 5.0f),

    AP_GROUPEND
};

AP_AllSpark::AP_AllSpark()
{
    singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
}

namespace AP {
    AP_AllSpark *allspark(void)
    {
        return AP_AllSpark::get_singleton();
    }
}

// little-endian writers used to build payloads explicitly
static void put_le16(uint8_t *p, uint16_t v)
{
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
}

static void put_le32(uint8_t *p, uint32_t v)
{
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
    p[2] = (v >> 16) & 0xFF;
    p[3] = (v >> 24) & 0xFF;
}

static void put_i16(uint8_t *p, int16_t v)
{
    put_le16(p, (uint16_t)v);
}

static void put_i32(uint8_t *p, int32_t v)
{
    put_le32(p, (uint32_t)v);
}

// a protocol field can be narrower than the value it carries; clamp instead of
// letting the cast wrap (a 400 deg/s roll rate would otherwise be reported as
// a small negative rate)
static int16_t clamp_i16(float v)
{
    return (int16_t)constrain_float(v, -32767.0f, 32767.0f);
}

// incoming message payloads (packed, little-endian)
struct PACKED GuidancePayload {
    uint16_t command_id;
    uint16_t valid_mask;
    uint8_t  altitude_reference;
    uint8_t  reserved;
    uint16_t gps_week;
    uint32_t gps_tow_ms;
    int32_t  target_latitude;      // deg * 1e7
    int32_t  target_longitude;     // deg * 1e7
    int32_t  target_altitude;      // cm
    uint16_t target_airspeed;      // m/s * 100
    int16_t  target_roll;          // deg * 100
    int16_t  target_climb_angle;   // deg * 100
    uint16_t timeout_ms;
};

struct PACKED FaultPayload {
    uint16_t command_id;
    uint8_t  action;
    uint8_t  fault_slot;
    uint8_t  fault_type;
    uint8_t  fault_location;
    uint16_t severity;
    int32_t  parameter_1;          // physical * 10000
    int32_t  parameter_2;
    uint16_t ramp_in_ms;
    uint16_t ramp_out_ms;
    uint32_t start_delay_ms;
    uint32_t duration_ms;
};

// compile-time verification that the on-wire layout matches the protocol spec
static_assert(sizeof(GuidancePayload) == ALLSPARK_GUIDANCE_PAYLOAD_LEN, "AllSpark guidance payload size mismatch");
static_assert(sizeof(FaultPayload) == ALLSPARK_FAULT_PAYLOAD_LEN, "AllSpark fault payload size mismatch");

// fault command actions (protocol section 6)
static constexpr uint8_t FAULT_ACTION_CLEAR   = 0;
static constexpr uint8_t FAULT_ACTION_INJECT  = 1;
static constexpr uint8_t FAULT_ACTION_UPDATE  = 2;
static constexpr uint8_t FAULT_ACTION_RESET   = 3;
static constexpr uint8_t FAULT_SLOT_AUTO      = 0xFF;

static constexpr uint8_t FAULT_TYPE_MAX = 13;
static constexpr uint8_t FAULT_LOC_MAX = 6;
static constexpr uint32_t FAULT_MAX_TIME_MS = 3600UL * 1000UL;

// guidance sanity limits, so a corrupt or hostile frame cannot command the
// aircraft somewhere absurd
static constexpr int32_t GUID_LAT_LIMIT   = 900000000;    // deg * 1e7
static constexpr int32_t GUID_LON_LIMIT   = 1800000000;   // deg * 1e7
static constexpr int32_t GUID_ALT_MIN_CM  = -100000;      // -1000 m
static constexpr int32_t GUID_ALT_MAX_CM  = 3000000;      // 30000 m
static constexpr uint16_t GUID_AIRSPEED_MAX_CMS = 10000;  // 100 m/s
static constexpr int16_t GUID_ANGLE_LIMIT_CD = 9000;      // 90 deg

// initialise: find the UART configured with SerialProtocol_AllSpark.
void AP_AllSpark::init(void)
{
    if (!enable) {
        return;
    }

    AP_SerialManager &serial_manager = AP::serialmanager();

    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_AllSpark, 0);
    if (_port != nullptr) {
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
        _port->begin(115200, 256, 256);
        // re-assert after begin(): BRD_SERn_RTSCTS / board_setup_uart() may have
        // enabled hardware flow control before we got here; a 3-wire adapter
        // leaves CTS floating high and would block RX otherwise.
        _port->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    }

    // JSBSim HITL link (SERIALn_PROTOCOL = 36, AHRS): we forward fault commands
    // to the model (JBF1), receive JBA1 receipts and send JBO1 actuator outputs.
    //
    // The port itself is owned by the JSBSim ExternalAHRS backend, which calls
    // begin(), configures flow control and is the single reader. Here we only
    // take a write-only pointer to it, and only when that backend is actually
    // the one driving the link.
    _jbs_port = nullptr;
#if AP_EXTERNAL_AHRS_ENABLED
    if (AP::externalAHRS().get_type() == AP_ExternalAHRS::DevType::JSBSim) {
        _jbs_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    }
#endif

    _jbs_token = 0;
    _jbs_txn.in_flight = false;
    _guidance_active = false;
    _guidance_timeout_ms = 0;
    _last_guidance_id = 0;
    _last_fault_id = 0;
    _last_fault_valid = false;
    _last_fault_result = ALLSPARK_ACK_RESULT_SUCCESS;
    _last_fault_error = ALLSPARK_ACK_ERR_NONE;
    _rx_bytes = 0;
    _rx_ok = 0;
    _rx_crc_errors = 0;
    _rx_dst_errors = 0;
    _rx_len_errors = 0;

    reset_parser();
    // no register_io_process(): update() runs from the vehicle's scheduler so
    // that guidance and fault handling happen on the same thread as the
    // flight loop (see the comment on update() in the header).
}

// our own MAVLink system id (SYSID_THISMAV) used as frame SRC_ID
uint8_t AP_AllSpark::src_id(void) const
{
    return (uint8_t)mavlink_system.sysid;
}

uint16_t AP_AllSpark::crc16_ccitt_init(const uint8_t *buf, uint16_t len, uint16_t crc)
{
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

uint16_t AP_AllSpark::crc16_ccitt(const uint8_t *buf, uint16_t len)
{
    return crc16_ccitt_init(buf, len, 0xFFFF);
}

void AP_AllSpark::reset_parser(void)
{
    _parse_state = ParseState::WAIT_HEAD1;
    _rx_ofs = 0;
    _rx_payload_len = 0;
}

// main entry point, called from the vehicle scheduler (main thread)
void AP_AllSpark::update(void)
{
    // drain AllSpark rx (independent of the JBS link)
    if (_port != nullptr) {
        // ChibiOS UARTDriver restricts available()/read() to the thread that
        // owns the port. This runs on the main thread, which is where the
        // serial manager opened it, so the check is only a safety net.
        if (!_port->is_owned_by_current_thread()) {
            _port->begin(0, 0, 0);
        }
        int16_t avail = _port->available();
        while (avail-- > 0) {
            uint8_t b;
            if (!_port->read(b)) {
                break;
            }
            process_byte(b);
        }
    }

    // JSBSim HITL link: actuator output + fault-command resend/timeout.
    // NOTE: the JBS *read* path (JBS2 sensor + JBA1 receipt) is owned by the
    // JSBSimAHRS ExternalAHRS backend, which is the single reader of the port.
    // AP_AllSpark only WRITES JBO1 (actuators) and JBF1 (fault) here.
    if (_jbs_port != nullptr) {
        jbs_tick();
        const uint32_t out_now = AP_HAL::millis();
        if (out_now - _last_output_ms >= 20) {   // 50 Hz actuator output
            _last_output_ms = out_now;
            send_actuator_output();
        }
    }

    if (_port == nullptr) {
        return;
    }

    check_guidance_timeout();

    // periodic state frame (20 Hz)
    const uint32_t now = AP_HAL::millis();
    if (now - _last_state_ms >= STATE_PERIOD_MS) {
        _last_state_ms = now;
        send_aircraft_state();
    }

    // diagnostic: report received bytes and why frames were discarded every 5s.
    // A frame rejected for version/CRC/address/length is otherwise silent, so
    // these counters are the quickest way to tell "nothing arrives" from
    // "arrives but is dropped".
    if (now - _last_rx_report_ms >= 5000) {
        _last_rx_report_ms = now;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AllSpark RX %u ok %u crc %u dst %u len %u",
                      (unsigned)_rx_bytes, (unsigned)_rx_ok, (unsigned)_rx_crc_errors,
                      (unsigned)_rx_dst_errors, (unsigned)_rx_len_errors);
    }
}

// Failsafe: an AllSpark guidance command is only valid for its own timeout (or
// the ALLSPK_TIMEOUT default). Once it goes stale the AllSpark targets are
// dropped, so a dropped link cannot leave the aircraft chasing an old command.
// The flight mode is deliberately left alone: the operator owns it, and an RC
// mode switch would simply re-select GUIDED anyway.
void AP_AllSpark::check_guidance_timeout(void)
{
    if (!_guidance_active || _guidance_timeout_ms == 0) {
        return;
    }
    if (AP_HAL::millis() - _last_guidance_ms < _guidance_timeout_ms) {
        return;
    }
    _guidance_active = false;
    AP::vehicle()->clear_allspark_guidance();
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AllSpark: guidance timeout, targets dropped");
}

void AP_AllSpark::process_byte(uint8_t b)
{
    _rx_bytes++;
    switch (_parse_state) {
    case ParseState::WAIT_HEAD1:
        if (b == HEAD1) {
            _rx_buf[0] = b;
            _rx_ofs = 1;
            _parse_state = ParseState::WAIT_HEAD2;
        }
        break;

    case ParseState::WAIT_HEAD2:
        if (b == HEAD2) {
            _rx_buf[1] = b;
            _rx_ofs = 2;
            _parse_state = ParseState::WAIT_HEADER;
        } else if (b != HEAD1) {
            reset_parser();
        } else {
            // could be the start of a new frame
            _rx_buf[0] = b;
            _rx_ofs = 1;
        }
        break;

    case ParseState::WAIT_HEADER:
        _rx_buf[_rx_ofs++] = b;
        if (_rx_ofs == 14) {
            _rx_payload_len = _rx_buf[8] | ((uint16_t)_rx_buf[9] << 8);
            if (_rx_payload_len > ALLSPARK_STATE_PAYLOAD_LEN) {
                _rx_len_errors++;
                reset_parser();
            } else {
                _parse_state = ParseState::WAIT_PAYLOAD;
            }
        }
        break;

    case ParseState::WAIT_PAYLOAD:
        _rx_buf[_rx_ofs++] = b;
        if (_rx_ofs >= 14 + _rx_payload_len + 3) {
            // complete frame present
            handle_frame(_rx_buf, _rx_payload_len);
            reset_parser();
        }
        break;
    }
}

void AP_AllSpark::handle_frame(const uint8_t *frame, uint16_t payload_len)
{
    // validate version and tail
    if (frame[2] != VERSION || frame[16 + payload_len] != TAIL) {
        _rx_crc_errors++;
        return;
    }

    // validate CRC over VERSION..end of payload (offset 2 .. 14+len-1)
    const uint16_t crc_calc = crc16_ccitt(&frame[2], 12 + payload_len);
    const uint16_t crc_rx = frame[14 + payload_len] | ((uint16_t)frame[15 + payload_len] << 8);
    if (crc_calc != crc_rx) {
        _rx_crc_errors++;
        return;
    }

    // only accept frames addressed to us (broadcast or our sysid)
    const uint8_t dst_id = frame[5];
    if (dst_id != DST_BROADCAST && dst_id != src_id()) {
        _rx_dst_errors++;
        return;
    }

    _rx_ok++;

    const uint8_t msg_id = frame[3];
    const uint8_t flags = frame[6];
    const uint8_t seq = frame[7];
    const uint8_t *payload = &frame[14];

    switch (msg_id) {
    case ALLSPARK_MSG_GUIDANCE_COMMAND:
        handle_guidance_command(payload, payload_len, flags, seq);
        break;
    case ALLSPARK_MSG_FAULT_COMMAND:
        handle_fault_command(payload, payload_len, flags, seq);
        break;
    default:
        break;
    }
}

void AP_AllSpark::handle_guidance_command(const uint8_t *payload, uint16_t payload_len, uint8_t flags, uint8_t seq)
{
    if (payload_len != ALLSPARK_GUIDANCE_PAYLOAD_LEN) {
        send_ack(seq, ALLSPARK_MSG_GUIDANCE_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, 0, ALLSPARK_ACK_ERR_BAD_LEN);
        return;
    }

    GuidancePayload g;
    memcpy(&g, payload, sizeof(g));

    // duplicate command id => ACK the cached result without re-applying
    if (g.command_id != 0 && g.command_id == _last_guidance_id) {
        send_ack(seq, ALLSPARK_MSG_GUIDANCE_COMMAND, ALLSPARK_ACK_RESULT_DUPLICATE, g.command_id, ALLSPARK_ACK_ERR_NONE);
        return;
    }

    // The protocol carries no authentication field; the trust boundary is the
    // serial port itself (SERIALn_PROTOCOL=51 plus ALLSPK_ENABLE). Two further
    // guards keep a stray device on the port from flying the aircraft:
    //   - guidance is only applied while the vehicle is already in GUIDED
    //     (protocol section 5); the FC never changes mode or arms itself here
    //   - a disarmed vehicle refuses guidance unless the operator opts in
    if (!hal.util->get_soft_armed() && !option_is_set(Options::ALLOW_UNARMED_GUIDANCE)) {
        send_ack(seq, ALLSPARK_MSG_GUIDANCE_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, g.command_id, ALLSPARK_ACK_ERR_SAFETY_STATE);
        return;
    }

    const bool have_llh   = (g.valid_mask & ALLSPARK_GUID_VALID_LLH) != 0;
    const bool have_alt   = (g.valid_mask & ALLSPARK_GUID_VALID_ALT) != 0;
    const bool have_airspeed = (g.valid_mask & ALLSPARK_GUID_VALID_AIRSPEED) != 0;
    const bool have_roll  = (g.valid_mask & ALLSPARK_GUID_VALID_ROLL) != 0;
    const bool have_climb = (g.valid_mask & ALLSPARK_GUID_VALID_CLIMB) != 0;

    if (!have_llh && !have_alt && !have_airspeed && !have_roll && !have_climb) {
        // nothing actionable in this command
        send_ack(seq, ALLSPARK_MSG_GUIDANCE_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, g.command_id, ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE);
        return;
    }

    AllSparkGuidance cmd {};
    cmd.airspeed_mps = -1.0f;

    if (have_llh) {
        if (g.target_latitude < -GUID_LAT_LIMIT || g.target_latitude > GUID_LAT_LIMIT ||
            g.target_longitude < -GUID_LON_LIMIT || g.target_longitude > GUID_LON_LIMIT ||
            (g.target_latitude == 0 && g.target_longitude == 0)) {
            send_ack(seq, ALLSPARK_MSG_GUIDANCE_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, g.command_id, ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE);
            return;
        }
        if (g.altitude_reference > 1) {
            send_ack(seq, ALLSPARK_MSG_GUIDANCE_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, g.command_id, ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE);
            return;
        }
        cmd.have_position = true;
        cmd.loc.lat = g.target_latitude;
        cmd.loc.lng = g.target_longitude;
    }

    if (have_alt) {
        if (g.target_altitude < GUID_ALT_MIN_CM || g.target_altitude > GUID_ALT_MAX_CM) {
            send_ack(seq, ALLSPARK_MSG_GUIDANCE_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, g.command_id, ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE);
            return;
        }
        // an altitude without a lat/lon is not a waypoint; ignore it
        if (cmd.have_position) {
            cmd.loc.set_alt_cm(g.target_altitude,
                               (g.altitude_reference == 1) ? Location::AltFrame::ABSOLUTE : Location::AltFrame::ABOVE_HOME);
            cmd.have_altitude = true;
        }
    }

    if (have_airspeed) {
        if (g.target_airspeed > GUID_AIRSPEED_MAX_CMS) {
            send_ack(seq, ALLSPARK_MSG_GUIDANCE_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, g.command_id, ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE);
            return;
        }
        cmd.airspeed_mps = g.target_airspeed * 0.01f;
    }

    if (have_roll) {
        if (g.target_roll < -GUID_ANGLE_LIMIT_CD || g.target_roll > GUID_ANGLE_LIMIT_CD) {
            send_ack(seq, ALLSPARK_MSG_GUIDANCE_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, g.command_id, ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE);
            return;
        }
        cmd.have_roll = true;
        cmd.roll_deg = g.target_roll * 0.01f;
    }

    if (have_climb) {
        if (g.target_climb_angle < -GUID_ANGLE_LIMIT_CD || g.target_climb_angle > GUID_ANGLE_LIMIT_CD) {
            send_ack(seq, ALLSPARK_MSG_GUIDANCE_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, g.command_id, ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE);
            return;
        }
        cmd.have_climb = true;
        cmd.climb_deg = g.target_climb_angle * 0.01f;
    }

    // the vehicle refuses the command unless it is already in GUIDED
    switch (AP::vehicle()->set_allspark_guidance(cmd)) {
    case AllSparkGuidanceResult::OK:
        break;
    case AllSparkGuidanceResult::OUT_OF_RANGE:
        send_ack(seq, ALLSPARK_MSG_GUIDANCE_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, g.command_id, ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE);
        return;
    case AllSparkGuidanceResult::NOT_GUIDED:
    default:
        send_ack(seq, ALLSPARK_MSG_GUIDANCE_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, g.command_id, ALLSPARK_ACK_ERR_NOT_GUIDED);
        return;
    }

    // the command is live: arm the staleness timer
    _last_guidance_id = g.command_id;
    _last_guidance_ms = AP_HAL::millis();
    _guidance_timeout_ms = (g.timeout_ms != 0) ? g.timeout_ms : (uint32_t)(guidance_timeout_s.get() * 1000.0f);
    _guidance_active = true;

    send_ack(seq, ALLSPARK_MSG_GUIDANCE_COMMAND, ALLSPARK_ACK_RESULT_SUCCESS, g.command_id, ALLSPARK_ACK_ERR_NONE);
}

// Apply a fault command inside the flight controller. Used when there is no
// live JSBSim model to hand the fault to.
bool AP_AllSpark::apply_local_fault(const uint8_t *payload)
{
#if AP_FAULT_INJECTION_ENABLED
    AP_FaultInjection *fij = AP::fault_injection();
    if (fij == nullptr) {
        return false;
    }

    FaultPayload f;
    memcpy(&f, payload, sizeof(f));

    switch (f.action) {
    case FAULT_ACTION_RESET:
        fij->release_all();
        return true;

    case FAULT_ACTION_CLEAR:
        if (f.fault_slot >= AP_FaultInjection::NUM_SLOTS) {
            return false;
        }
        fij->release_slot(f.fault_slot);
        return true;

    case FAULT_ACTION_INJECT: {
        uint8_t slot = f.fault_slot;
        if (slot == FAULT_SLOT_AUTO) {
            slot = fij->find_free_slot();
            if (slot >= AP_FaultInjection::NUM_SLOTS) {
                return false;   // no free slot
            }
        } else if (slot >= AP_FaultInjection::NUM_SLOTS) {
            return false;
        } else if (fij->slot(slot).armed) {
            return false;       // slot occupied; the GCS must use UPDATE or CLEAR
        }
        return fij->set_fault(slot, AP_FaultInjection::Type(f.fault_type), f.fault_location,
                              f.parameter_1 * 0.0001f, f.parameter_2 * 0.0001f,
                              f.ramp_in_ms, f.ramp_out_ms, f.start_delay_ms, f.duration_ms);
    }

    case FAULT_ACTION_UPDATE:
        if (f.fault_slot >= AP_FaultInjection::NUM_SLOTS || !fij->slot(f.fault_slot).armed) {
            return false;       // cannot update an inactive slot
        }
        return fij->set_fault(f.fault_slot, AP_FaultInjection::Type(f.fault_type), f.fault_location,
                              f.parameter_1 * 0.0001f, f.parameter_2 * 0.0001f,
                              f.ramp_in_ms, f.ramp_out_ms, f.start_delay_ms, f.duration_ms);

    default:
        return false;
    }
#else
    (void)payload;
    return false;
#endif  // AP_FAULT_INJECTION_ENABLED
}

void AP_AllSpark::handle_fault_command(const uint8_t *payload, uint16_t payload_len, uint8_t flags, uint8_t seq)
{
    if (payload_len != ALLSPARK_FAULT_PAYLOAD_LEN) {
        send_ack(seq, ALLSPARK_MSG_FAULT_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, 0, ALLSPARK_ACK_ERR_BAD_LEN);
        return;
    }

    FaultPayload f;
    memcpy(&f, payload, sizeof(f));

    // Retransmission handling (protocol section 8): a command with the same id
    // and identical content must be answered from the cache and never applied
    // twice; the same id with different content is an error.
    if (_last_fault_valid && f.command_id != 0 && f.command_id == _last_fault_id) {
        if (memcmp(_last_fault_payload, payload, ALLSPARK_FAULT_PAYLOAD_LEN) != 0) {
            send_ack(seq, ALLSPARK_MSG_FAULT_COMMAND, ALLSPARK_ACK_RESULT_REJECTED,
                     f.command_id, ALLSPARK_ACK_ERR_DUP_ID_DIFF_CONTENT);
            return;
        }
        // a successful command is re-ACKed as DUPLICATE so the ground station
        // knows it was not applied a second time
        const uint8_t cached = (_last_fault_result == ALLSPARK_ACK_RESULT_SUCCESS)
                                 ? ALLSPARK_ACK_RESULT_DUPLICATE
                                 : _last_fault_result;
        send_ack(seq, ALLSPARK_MSG_FAULT_COMMAND, cached, f.command_id, _last_fault_error);
        return;
    }

    // validate per protocol section 6
    if (f.action > FAULT_ACTION_RESET) {
        send_ack(seq, ALLSPARK_MSG_FAULT_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, f.command_id, ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE);
        return;
    }
    // fault_slot must be 0..7, except:
    //   - inject (action=1) allows 0xFF for auto-select
    //   - reset-all (action=3) ignores the slot entirely
    if (f.action != FAULT_ACTION_RESET && f.fault_slot > 7 &&
        !(f.action == FAULT_ACTION_INJECT && f.fault_slot == FAULT_SLOT_AUTO)) {
        send_ack(seq, ALLSPARK_MSG_FAULT_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, f.command_id, ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE);
        return;
    }
    if (f.fault_type > FAULT_TYPE_MAX) {
        send_ack(seq, ALLSPARK_MSG_FAULT_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, f.command_id, ALLSPARK_ACK_ERR_FAULT_UNSUPPORTED);
        return;
    }
    if (f.fault_location > FAULT_LOC_MAX) {
        send_ack(seq, ALLSPARK_MSG_FAULT_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, f.command_id, ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE);
        return;
    }
    // surface faults (type 3/4/5) must name a surface (1/2/3)
    if ((f.fault_type == 3 || f.fault_type == 4 || f.fault_type == 5) &&
        (f.fault_location < 1 || f.fault_location > 3)) {
        send_ack(seq, ALLSPARK_MSG_FAULT_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, f.command_id, ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE);
        return;
    }
    // severity fixed at 1000; the magnitude is carried by P1
    if (f.severity != 1000) {
        send_ack(seq, ALLSPARK_MSG_FAULT_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, f.command_id, ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE);
        return;
    }
    // ramp-in only allowed for gradual faults (type 2/5)
    if (f.ramp_in_ms != 0 && !(f.fault_type == 2 || f.fault_type == 5)) {
        send_ack(seq, ALLSPARK_MSG_FAULT_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, f.command_id, ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE);
        return;
    }
    // time fields capped at 3600s
    if (f.ramp_in_ms > FAULT_MAX_TIME_MS || f.ramp_out_ms > FAULT_MAX_TIME_MS ||
        f.start_delay_ms > FAULT_MAX_TIME_MS || f.duration_ms > FAULT_MAX_TIME_MS) {
        send_ack(seq, ALLSPARK_MSG_FAULT_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, f.command_id, ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE);
        return;
    }

    // With a live JSBSim model the fault belongs to the model: it applies the
    // fault to its own dynamics and reports the resulting (corrupted) sensors
    // back over JBS2, so the FC must not apply it a second time. Without a
    // model - i.e. on the real aircraft - the FC applies it itself.
    if (_jbs_port != nullptr && AP::externalAHRS().healthy()) {
        if (_jbs_txn.in_flight) {
            // A retransmission of the command already in flight must stay
            // "pending", otherwise the ground station sees a rejection and
            // gives up while the model is still applying the fault.
            const bool same_command = (_jbs_txn.command_id == f.command_id);
            send_ack(seq, ALLSPARK_MSG_FAULT_COMMAND,
                     same_command ? ALLSPARK_ACK_RESULT_PENDING : ALLSPARK_ACK_RESULT_REJECTED,
                     f.command_id,
                     same_command ? ALLSPARK_ACK_ERR_NONE : ALLSPARK_ACK_ERR_FAULT_BUSY);
            return;
        }

        // start a new model-fault transaction and ACK "queued" (result=1)
        _jbs_txn.in_flight = true;
        _jbs_txn.token = ++_jbs_token;
        _jbs_txn.command_id = f.command_id;
        _jbs_txn.ack_seq = seq;
        _jbs_txn.result = ALLSPARK_ACK_RESULT_PENDING;
        _jbs_txn.error_code = ALLSPARK_ACK_ERR_NONE;
        _jbs_txn.start_ms = AP_HAL::millis();
        _jbs_txn.last_send_ms = 0;
        memcpy(_jbs_txn.fault_payload, payload, ALLSPARK_FAULT_PAYLOAD_LEN);
        send_jbf1(_jbs_txn.fault_payload, ALLSPARK_FAULT_PAYLOAD_LEN, _jbs_txn.token, src_id());
        send_ack(seq, ALLSPARK_MSG_FAULT_COMMAND, ALLSPARK_ACK_RESULT_PENDING, f.command_id, ALLSPARK_ACK_ERR_NONE);
        return;
    }

    if (!apply_local_fault(payload)) {
        send_ack(seq, ALLSPARK_MSG_FAULT_COMMAND, ALLSPARK_ACK_RESULT_REJECTED, f.command_id, ALLSPARK_ACK_ERR_FAULT_BUSY);
        return;
    }

    _last_fault_id = f.command_id;
    _last_fault_valid = true;
    _last_fault_result = ALLSPARK_ACK_RESULT_SUCCESS;
    _last_fault_error = ALLSPARK_ACK_ERR_NONE;
    memcpy(_last_fault_payload, payload, ALLSPARK_FAULT_PAYLOAD_LEN);
    send_ack(seq, ALLSPARK_MSG_FAULT_COMMAND, ALLSPARK_ACK_RESULT_SUCCESS, f.command_id, ALLSPARK_ACK_ERR_NONE);
}

void AP_AllSpark::send_frame(uint8_t msg_id, const uint8_t *payload, uint16_t payload_len, uint8_t flags)
{
    if (_port == nullptr) {
        return;
    }
    if (_port->txspace() < (uint16_t)(17 + payload_len)) {
        return;
    }

    uint8_t frame[17 + ALLSPARK_STATE_PAYLOAD_LEN];
    frame[0] = HEAD1;
    frame[1] = HEAD2;
    frame[2] = VERSION;
    frame[3] = msg_id;
    frame[4] = src_id();
    frame[5] = DST_ALLSPARK;
    frame[6] = flags;
    frame[7] = _seq++;
    put_le16(&frame[8], payload_len);
    put_le32(&frame[10], AP_HAL::millis());
    memcpy(&frame[14], payload, payload_len);

    const uint16_t crc = crc16_ccitt(&frame[2], 12 + payload_len);
    frame[14 + payload_len] = crc & 0xFF;
    frame[15 + payload_len] = (crc >> 8) & 0xFF;
    frame[16 + payload_len] = TAIL;

    _port->write(frame, 17 + payload_len);
}

void AP_AllSpark::send_ack(uint8_t ack_seq, uint8_t ack_msg_id, uint8_t result, uint16_t command_id, uint16_t error_code)
{
    uint8_t payload[ALLSPARK_ACK_PAYLOAD_LEN];
    payload[0] = ack_seq;
    payload[1] = ack_msg_id;
    payload[2] = result;
    payload[3] = 0;  // reserved
    put_le16(&payload[4], command_id);
    put_le16(&payload[6], error_code);
    send_frame(ALLSPARK_MSG_ACK, payload, ALLSPARK_ACK_PAYLOAD_LEN, 0);
}

void AP_AllSpark::send_aircraft_state(void)
{
    uint8_t payload[ALLSPARK_STATE_PAYLOAD_LEN];
    build_aircraft_state(payload);
    send_frame(ALLSPARK_MSG_AIRCRAFT_STATE, payload, ALLSPARK_STATE_PAYLOAD_LEN, 0);
}

void AP_AllSpark::build_aircraft_state(uint8_t *payload) const
{
    memset(payload, 0, ALLSPARK_STATE_PAYLOAD_LEN);

    uint32_t valid_mask = 0;
    uint16_t state_flags = 0;

    // sysid + flight mode
    payload[0] = src_id();
    payload[1] = AP::vehicle()->get_mode();

    // state_flags
    if (hal.util->get_soft_armed()) {
        state_flags |= 0x0001;
    }
    const AP_Notify &notify = AP::notify();
    if (notify.flags.failsafe_radio || notify.flags.failsafe_battery ||
        notify.flags.failsafe_gcs || notify.flags.failsafe_ekf) {
        state_flags |= 0x0002;
    }
    if (AP::gps().status() >= AP_GPS_FixType::FIX_3D) {
        state_flags |= 0x0004;
    }
    if (AP::ahrs().healthy()) {
        state_flags |= 0x0008;
    }
    // bit4: a fault is currently present (protocol section 7). The local engine
    // covers the no-model case; the model truth below can also set it.
#if AP_FAULT_INJECTION_ENABLED
    const AP_FaultInjection *fij = AP::fault_injection();
    if (fij != nullptr && fij->any_active()) {
        state_flags |= 0x0010;
    }
#endif
#if AP_SIM_ENABLED
    state_flags |= 0x0020;  // HITL/SITL
#endif
#if AP_EXTERNAL_AHRS_ENABLED
    // a real aircraft driven by the JSBSim model is HITL too; the AllSpark
    // ground station uses this bit to decide whether model faults can be sent
    if (AP::externalAHRS().get_type() == AP_ExternalAHRS::DevType::JSBSim) {
        state_flags |= 0x0020;
    }
#endif

    // GPS time
    put_le16(&payload[4], AP::gps().time_week());
    put_le32(&payload[6], AP::gps().time_week_ms());
    valid_mask |= 0x00000080;   // bit7 GPS time (always reported)

    // attitude (deg*100)
    put_i16(&payload[10], (int16_t)AP::ahrs().roll_sensor);
    put_i16(&payload[12], (int16_t)AP::ahrs().pitch_sensor);
    put_le16(&payload[14], (uint16_t)AP::ahrs().yaw_sensor);
    valid_mask |= 0x00000001;   // bit0 attitude

    // body rates (deg/s * 100)
    const Vector3f gyro = AP::ahrs().get_gyro();   // rad/s
    put_i16(&payload[16], clamp_i16(gyro.x * 5729.57795131f));
    put_i16(&payload[18], clamp_i16(gyro.y * 5729.57795131f));
    put_i16(&payload[20], clamp_i16(gyro.z * 5729.57795131f));
    valid_mask |= 0x00000002;   // bit1 rates

    // servo/motor commands (PWM us)
    uint16_t pwm;
    if (SRV_Channels::get_output_pwm(SRV_Channel::k_aileron, pwm)) { put_le16(&payload[22], pwm); }
    if (SRV_Channels::get_output_pwm(SRV_Channel::k_elevator, pwm)) { put_le16(&payload[24], pwm); }
    if (SRV_Channels::get_output_pwm(SRV_Channel::k_rudder, pwm)) { put_le16(&payload[26], pwm); }
    // payload[28..29] servo[3] = reserved (0)
    if (SRV_Channels::get_output_pwm(SRV_Channel::k_throttle, pwm)) { put_le16(&payload[30], pwm); }
    // payload[32..33] motor[1] = reserved (0)
    valid_mask |= 0x00000004;   // bit2 commands

    // current position
    Location loc;
    if (AP::ahrs().get_location(loc)) {
        put_i32(&payload[34], (int32_t)loc.lat);
        put_i32(&payload[38], (int32_t)loc.lng);
        put_i32(&payload[42], (int32_t)loc.alt);
        valid_mask |= 0x00000008;   // bit3 position
    }

    // ground speed (m/s * 100)
    if (AP::gps().status() >= AP_GPS_FixType::FIX_3D) {
        put_le16(&payload[46], (uint16_t)roundf(AP::gps().ground_speed() * 100.0f));
        valid_mask |= 0x00000010;   // bit4 ground speed
    }

    // airspeed (m/s * 100)
    if (AP::airspeed().enabled() && AP::airspeed().healthy()) {
        put_le16(&payload[48], (uint16_t)roundf(AP::airspeed().get_airspeed() * 100.0f));
        valid_mask |= 0x00000020;   // bit5 airspeed
    }

    // current navigation target
    Location target;
    if (AP::vehicle()->get_allspark_target_location(target)) {
        put_i32(&payload[50], (int32_t)target.lat);
        put_i32(&payload[54], (int32_t)target.lng);
        put_i32(&payload[58], (int32_t)target.alt);
        valid_mask |= 0x00000040;   // bit6 target
    }

    // battery remaining (% * 10)
    uint8_t pct;
    if (AP::battery().capacity_remaining_pct(pct)) {
        put_le16(&payload[62], (uint16_t)pct * 10);
        valid_mask |= 0x00000100;   // bit8 battery
    }

    // climb angle (deg * 100) from NED velocity
    Vector3f vel;
    if (AP::ahrs().get_velocity_NED(vel)) {
        const float hspeed = sqrtf(vel.x * vel.x + vel.y * vel.y);
        if (hspeed > 1.0f) {
            const float climb_deg = atan2f(-vel.z, hspeed) * 57.2957795131f;
            put_i16(&payload[64], (int16_t)roundf(climb_deg * 100.0f));
            valid_mask |= 0x00000200;   // bit9 climb angle
        }
    }

    // model truth (AoA / sideslip / real surfaces / fault) from JSBSim JBS2
    bool fault_reported = false;
#if AP_EXTERNAL_AHRS_ENABLED
    const AP_ExternalAHRS::JSBSimTruth &truth = AP::externalAHRS().get_jsbsim_truth();
    // truth.valid is latched the first time a JBS2 frame arrives and is never
    // cleared, so it must be combined with the link being fresh; otherwise a
    // dropped model link would keep reporting stale attitude/fault data
    if (truth.valid && AP::externalAHRS().healthy()) {
        fault_reported = true;
        // the model owns the validity of these fields; do not claim data it
        // says it does not have (protocol section 7: "无真实舵机反馈或油门
        // 反馈时必须清除 bit12/bit13")
        if ((truth.valid_mask & (1U << 0)) != 0) {
            put_i16(&payload[66], clamp_i16(truth.aoa_deg * 100.0f));
            put_i16(&payload[68], clamp_i16(truth.sideslip_deg * 100.0f));
            valid_mask |= 0x00000400;   // bit10 AoA
            valid_mask |= 0x00000800;   // bit11 sideslip
        }

        if ((truth.valid_mask & (1U << 12)) != 0) {
            for (uint8_t i = 0; i < 4; i++) {
                put_i16(&payload[70 + i * 2], truth.actual_surface_cd[i]);
            }
            valid_mask |= 0x00001000;   // bit12 actual surface
        }

        if ((truth.valid_mask & (1U << 13)) != 0) {
            put_le16(&payload[78], truth.actual_throttle_permille[0]);
            put_le16(&payload[80], truth.actual_throttle_permille[1]);
            valid_mask |= 0x00002000;   // bit13 actual throttle
        }

        payload[82] = truth.active_fault_count;
        payload[83] = truth.fault_slot;
        payload[84] = truth.fault_type;
        payload[85] = truth.fault_location;
        payload[86] = truth.fault_state;
        payload[87] = truth.active_fault_mask;
        put_le16(&payload[88], truth.severity_permille);
        put_i32(&payload[90], truth.parameter_1_q10000);
        put_i32(&payload[94], truth.parameter_2_q10000);
        valid_mask |= 0x00004000;   // bit14 fault truth
        if (truth.active_fault_count > 0) {
            state_flags |= 0x0010;  // bit4: a fault is present
        }
    }
#endif  // AP_EXTERNAL_AHRS_ENABLED

#if AP_FAULT_INJECTION_ENABLED
    if (!fault_reported) {
        // No model truth: report the flight controller's own engine, otherwise
        // a locally injected fault would be completely invisible to the ground
        // station. Without model truth there is no real surface/throttle
        // feedback, so bits 12/13 stay clear.
        const AP_FaultInjection *local_fij = AP::fault_injection();
        if (local_fij != nullptr && local_fij->any_active()) {
            uint8_t first_slot = 0xFF;
            uint8_t active_mask = 0;
            uint8_t count = 0;
            for (uint8_t i = 0; i < AP_FaultInjection::NUM_SLOTS; i++) {
                if (!local_fij->slot(i).armed) {
                    continue;
                }
                count++;
                active_mask |= (uint8_t)(1U << i);
                if (first_slot == 0xFF) {
                    first_slot = i;
                }
            }
            const AP_FaultInjection::FaultState &f = local_fij->slot(first_slot);
            payload[82] = count;
            payload[83] = first_slot;
            payload[84] = (uint8_t)f.type;
            payload[85] = f.location;
            // 2 = waiting for the start delay, 3 = ramping in, 4 = applied
            payload[86] = f.active ? ((f.ramp_progress < 1.0f && f.ramp_in_ms != 0) ? 3 : 4) : 2;
            payload[87] = active_mask;
            put_le16(&payload[88], (uint16_t)roundf(f.current_severity * 1000.0f));
            put_i32(&payload[90], (int32_t)roundf(f.p1 * 10000.0f));
            put_i32(&payload[94], (int32_t)roundf(f.p2 * 10000.0f));
            valid_mask |= 0x00004000;   // bit14 fault truth
            state_flags |= 0x0010;      // bit4: a fault is present
        }
    }
#endif  // AP_FAULT_INJECTION_ENABLED

    // state_flags is written last because the model truth can set bit4 too
    put_le16(&payload[2], state_flags);

    put_le32(&payload[98], valid_mask);
}

// ---- JSBSim HITL (JBS) internal frames ----
// Frame: magic[4] + payload_len u16(LE) + payload + CRC16 u16(LE)
// CRC poly 0x1021, init 0, over magic..payload (protocol doc section 11).

void AP_AllSpark::send_jbs_frame(const char *magic, const uint8_t *payload, uint16_t payload_len)
{
    if (_jbs_port == nullptr) {
        return;
    }
    if (_jbs_port->txspace() < (uint16_t)(8 + payload_len)) {
        return;
    }
    uint8_t buf[8 + 64];
    memcpy(&buf[0], magic, 4);
    put_le16(&buf[4], payload_len);
    memcpy(&buf[6], payload, payload_len);
    const uint16_t crc = crc16_ccitt_init(buf, 6 + payload_len, 0);
    put_le16(&buf[6 + payload_len], crc);
    _jbs_port->write(buf, 8 + payload_len);
}

// JBF1: FC -> model fault command. payload = token u32 + mav_sysid u8 + allspark_source u8 + raw 28-byte fault
void AP_AllSpark::send_jbf1(const uint8_t *fault_payload, uint16_t fault_len, uint32_t token, uint8_t source)
{
    uint8_t payload[34];
    put_le32(&payload[0], token);
    payload[4] = src_id();      // mav_sysid
    payload[5] = source;        // allspark source (AllSpark fixed id 0x80)
    memcpy(&payload[6], fault_payload, MIN(fault_len, (uint16_t)ALLSPARK_FAULT_PAYLOAD_LEN));
    send_jbs_frame("JBF1", payload, 34);
}

// JBO1: FC -> model actuator outputs. payload = time_ms u32 + 4x PWM u16
// order: aileron, elevator, throttle, rudder (matches JSBSim apply_pwm_outputs).
void AP_AllSpark::send_actuator_output(void)
{
    if (_jbs_port == nullptr) {
        return;
    }
    uint16_t pwm[4] = {1500, 1500, 1000, 1500};  // neutral surfaces, idle throttle
    uint16_t v;
    if (SRV_Channels::get_output_pwm(SRV_Channel::k_aileron, v))  pwm[0] = v;
    if (SRV_Channels::get_output_pwm(SRV_Channel::k_elevator, v)) pwm[1] = v;
    if (SRV_Channels::get_output_pwm(SRV_Channel::k_throttle, v)) pwm[2] = v;
    if (SRV_Channels::get_output_pwm(SRV_Channel::k_rudder, v))   pwm[3] = v;

    // JBO1 bypasses the HAL output stage, so apply its arming/safety gate here.
    // The model protocol defines 1000 as motor off, independently of SERVOx_MIN.
    if (!AP::arming().is_armed_and_safety_off()) {
        pwm[2] = 1000;
    }

    uint8_t payload[12];
    put_le32(&payload[0], AP_HAL::millis());
    put_le16(&payload[4], pwm[0]);
    put_le16(&payload[6], pwm[1]);
    put_le16(&payload[8], pwm[2]);
    put_le16(&payload[10], pwm[3]);
    send_jbs_frame("JBO1", payload, 12);
}

void AP_AllSpark::jbs_tick(void)
{
    if (!_jbs_txn.in_flight) {
        return;
    }
    const uint32_t now = AP_HAL::millis();
    if (now - _jbs_txn.start_ms >= JBS_TIMEOUT_MS) {
        jbs_ack_complete(ALLSPARK_ACK_RESULT_FAILED, ALLSPARK_ACK_ERR_MODEL_NOT_READY,
                         _jbs_txn.command_id, _jbs_txn.ack_seq);
        return;
    }
    if (now - _jbs_txn.last_send_ms >= JBS_RESEND_MS) {
        _jbs_txn.last_send_ms = now;
        send_jbf1(_jbs_txn.fault_payload, ALLSPARK_FAULT_PAYLOAD_LEN, _jbs_txn.token, src_id());
    }
}

// JBA1: model -> FC receipt. payload = token u32 + mav_sysid u8 + source u8 + result u8 + error_code u16
void AP_AllSpark::handle_jba1(const uint8_t *payload, uint16_t payload_len)
{
    if (payload_len != 9 || !_jbs_txn.in_flight) {
        return;
    }
    uint32_t token;
    memcpy(&token, &payload[0], 4);
    const uint8_t sysid = payload[4];
    const uint8_t result = payload[6];
    const uint16_t error_code = payload[7] | ((uint16_t)payload[8] << 8);

    // must match our in-flight transaction token and sysid
    if (token != _jbs_txn.token || sysid != src_id()) {
        return;
    }

    // the model result uses the same encoding as the AllSpark ACK result
    switch (result) {
    case ALLSPARK_ACK_RESULT_SUCCESS:
    case ALLSPARK_ACK_RESULT_DUPLICATE:
    case ALLSPARK_ACK_RESULT_REJECTED:
    case ALLSPARK_ACK_RESULT_FAILED:
        break;
    case ALLSPARK_ACK_RESULT_PENDING:
        // still working: keep the transaction open, the 2s timeout applies
        return;
    default:
        jbs_ack_complete(ALLSPARK_ACK_RESULT_FAILED, error_code, _jbs_txn.command_id, _jbs_txn.ack_seq);
        return;
    }

    jbs_ack_complete(result, error_code, _jbs_txn.command_id, _jbs_txn.ack_seq);
}

void AP_AllSpark::jbs_ack_complete(uint8_t result, uint16_t error_code, uint16_t command_id, uint8_t ack_seq)
{
    _jbs_txn.in_flight = false;

    // cache the outcome so a retransmission of this command is answered from
    // the cache instead of being forwarded to the model again
    _last_fault_id = command_id;
    _last_fault_result = result;
    _last_fault_error = error_code;
    _last_fault_valid = true;
    memcpy(_last_fault_payload, _jbs_txn.fault_payload, ALLSPARK_FAULT_PAYLOAD_LEN);

    send_ack(ack_seq, ALLSPARK_MSG_FAULT_COMMAND, result, command_id, error_code);
}

#endif  // AP_ALLSPARK_ENABLED
