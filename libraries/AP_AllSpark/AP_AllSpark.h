/*
  AP_AllSpark - Allspark <-> flight controller binary serial protocol

  Implements the "Allspark 与 CUAV V5+ 二进制通信协议" over a dedicated
  UART (SERIALn_PROTOCOL = 51).

  Frame layout (all multi-byte fields little-endian):
    offset  0 : HEAD1        = 0xAB
    offset  1 : HEAD2        = 0xBA
    offset  2 : VERSION      = 0x01
    offset  3 : MSG_ID
    offset  4 : SRC_ID       (flight controller side == MAV_SYSID)
    offset  5 : DST_ID       (0 = broadcast, 0x80 = Allspark)
    offset  6 : FLAGS        (bit0 req-ACK, bit1 ACK, bit2 NACK)
    offset  7 : SEQ          (increments every frame)
    offset  8 : PAYLOAD_LEN  (uint16)
    offset 10 : TIME_MS      (uint32, ms since sender boot)
    offset 14 : PAYLOAD      (PAYLOAD_LEN bytes)
    offset 14+N : CRC16      (uint16, low byte first)
    offset 16+N : TAIL       = 0xFF

  CRC16-CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final xor;
  computed over VERSION .. end of PAYLOAD (excludes header, CRC and tail).
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#include "AP_AllSpark_config.h"

#if AP_ALLSPARK_ENABLED

// message IDs
#define ALLSPARK_MSG_ACK               0x02
#define ALLSPARK_MSG_GUIDANCE_COMMAND  0x10
#define ALLSPARK_MSG_FAULT_COMMAND     0x11
#define ALLSPARK_MSG_AIRCRAFT_STATE    0x20

// payload lengths
#define ALLSPARK_ACK_PAYLOAD_LEN        8
#define ALLSPARK_GUIDANCE_PAYLOAD_LEN   32
#define ALLSPARK_FAULT_PAYLOAD_LEN      28
#define ALLSPARK_STATE_PAYLOAD_LEN      102

// ACK result codes
#define ALLSPARK_ACK_RESULT_SUCCESS        0
#define ALLSPARK_ACK_RESULT_PENDING        1
#define ALLSPARK_ACK_RESULT_DUPLICATE      2
#define ALLSPARK_ACK_RESULT_REJECTED       3
#define ALLSPARK_ACK_RESULT_FAILED         4

// ACK error codes
#define ALLSPARK_ACK_ERR_NONE              0
#define ALLSPARK_ACK_ERR_BAD_VERSION       1
#define ALLSPARK_ACK_ERR_BAD_LEN           2
#define ALLSPARK_ACK_ERR_BAD_CRC           3
#define ALLSPARK_ACK_ERR_BAD_DST           4
#define ALLSPARK_ACK_ERR_PARAM_OUT_OF_RANGE 5
#define ALLSPARK_ACK_ERR_NOT_GUIDED        6
// protocol V1.1 error 7: "安全状态不允许" (the safety state does not allow it)
#define ALLSPARK_ACK_ERR_SAFETY_STATE      7
#define ALLSPARK_ACK_ERR_FAULT_BUSY        8
#define ALLSPARK_ACK_ERR_FAULT_UNSUPPORTED 9
#define ALLSPARK_ACK_ERR_MODEL_NOT_READY   10
#define ALLSPARK_ACK_ERR_DUP_ID_DIFF_CONTENT 11

// guidance valid_mask bits
#define ALLSPARK_GUID_VALID_GPS_TIME   (1U<<0)
#define ALLSPARK_GUID_VALID_LLH        (1U<<1)
#define ALLSPARK_GUID_VALID_ALT        (1U<<2)
#define ALLSPARK_GUID_VALID_AIRSPEED   (1U<<3)
#define ALLSPARK_GUID_VALID_ROLL       (1U<<4)
#define ALLSPARK_GUID_VALID_CLIMB      (1U<<5)

class AP_AllSpark {
public:
    AP_AllSpark();
    CLASS_NO_COPY(AP_AllSpark);

    static AP_AllSpark *get_singleton(void) { return singleton; }

    // parameter table
    static const struct AP_Param::GroupInfo var_info[];

    // ALLSPK_OPTIONS bits
    enum class Options : uint8_t {
        ALLOW_UNARMED_GUIDANCE = 1U << 0,
    };
    bool option_is_set(Options option) const {
        return (options.get() & uint8_t(option)) != 0;
    }

    // initialise: locate the UART configured with SerialProtocol_AllSpark
    void init(void);

    // periodic work: drain the UART, run the JBS link and send the state frame.
    //
    // This MUST be called from the vehicle's main (scheduler) thread, not from
    // the HAL IO thread. It applies guidance to the navigation state and writes
    // the fault-injection slots, both of which the flight loop reads: acting
    // from the IO thread races with navigation. Estimated < 300us per call.
    void update(void);

    // route a JBA1 receipt payload to the fault transaction. Called by the
    // JSBSimAHRS ExternalAHRS backend, which owns the JBS serial read path.
    void handle_jba1_payload(const uint8_t *payload, uint16_t payload_len) {
        handle_jba1(payload, payload_len);
    }

private:
    static AP_AllSpark *singleton;

    AP_HAL::UARTDriver *_port;

    AP_Int8  enable;
    AP_Int8  options;
    AP_Float guidance_timeout_s;   // 0 disables the AllSpark guidance timeout

    // ---- frame markers ----
    static constexpr uint8_t HEAD1 = 0xAB;
    static constexpr uint8_t HEAD2 = 0xBA;
    static constexpr uint8_t VERSION = 0x01;
    static constexpr uint8_t TAIL = 0xFF;
    static constexpr uint8_t DST_BROADCAST = 0x00;
    static constexpr uint8_t DST_ALLSPARK = 0x80;

    // state frame rate (20 Hz default)
    static constexpr uint32_t STATE_PERIOD_MS = 50;

    // receive parser state
    enum class ParseState : uint8_t {
        WAIT_HEAD1,
        WAIT_HEAD2,
        WAIT_HEADER,
        WAIT_PAYLOAD,
    };

    ParseState _parse_state;
    uint16_t _rx_ofs;
    uint16_t _rx_payload_len;
    // worst-case frame: 17 byte header/crc/tail + 102 byte payload
    uint8_t _rx_buf[17 + ALLSPARK_STATE_PAYLOAD_LEN + 2];

    // transmit sequencing
    uint8_t _seq;
    uint32_t _last_state_ms;
    uint32_t _last_output_ms;

    // diagnostic: total RX bytes received (for HIL bring-up)
    uint32_t _rx_bytes;
    uint32_t _last_rx_report_ms;
    // why frames were discarded - a rejected frame is otherwise silent, which
    // makes bring-up guesswork. Reported every 5s next to the byte count.
    uint32_t _rx_ok;            // frames accepted and dispatched
    uint32_t _rx_crc_errors;    // version/tail/CRC failure
    uint32_t _rx_dst_errors;    // addressed to another node
    uint32_t _rx_len_errors;    // payload length out of range

    // last accepted guidance command (duplicate detection + timeout)
    uint16_t _last_guidance_id;
    uint32_t _last_guidance_ms;
    uint32_t _guidance_timeout_ms;    // 0 == no timeout
    bool     _guidance_active;

    // last accepted fault command (duplicate detection + cached outcome).
    // The protocol requires a retransmitted command to be answered from the
    // cache instead of being applied again.
    uint16_t _last_fault_id;
    bool     _last_fault_valid;
    uint8_t  _last_fault_result;   // cached ACK result for that command
    uint16_t _last_fault_error;
    uint8_t  _last_fault_payload[ALLSPARK_FAULT_PAYLOAD_LEN];

    // ---- JBS (JSBSim HITL) fault-forwarding link (protocol 36 = AHRS) ----
    // Internal frames use: magic[4] + payload_len u16(LE) + payload + CRC16 u16(LE),
    // CRC poly 0x1021 init 0 over magic..payload (protocol doc section 11).
    //
    // The port is OWNED by the JSBSim ExternalAHRS backend: that backend calls
    // begin(), sets flow control and is the only reader. AP_AllSpark only
    // writes JBO1/JBF1 here.
    AP_HAL::UARTDriver *_jbs_port;
    uint32_t _jbs_token;
    // single in-flight model fault transaction
    struct {
        bool in_flight;
        uint32_t token;
        uint16_t command_id;
        uint8_t ack_seq;        // SEQ of the command we must ACK
        uint8_t result;         // final ACK result (filled on JBA1 / timeout)
        uint16_t error_code;
        uint32_t last_send_ms;  // for 100ms resend
        uint32_t start_ms;      // for 2s timeout
        uint8_t fault_payload[ALLSPARK_FAULT_PAYLOAD_LEN];  // raw payload for resend
    } _jbs_txn;

    static constexpr uint32_t JBS_RESEND_MS = 100;
    static constexpr uint32_t JBS_TIMEOUT_MS = 2000;

    // parser
    void process_byte(uint8_t b);
    void reset_parser(void);
    void handle_frame(const uint8_t *frame, uint16_t payload_len);

    // message handlers (incoming)
    void handle_guidance_command(const uint8_t *payload, uint16_t payload_len, uint8_t flags, uint8_t seq);
    void handle_fault_command(const uint8_t *payload, uint16_t payload_len, uint8_t flags, uint8_t seq);

    // guidance timeout / failsafe
    void check_guidance_timeout(void);

    // apply a fault command inside the flight controller (no model link).
    // payload is the raw 28-byte ALLSPARK_MSG_FAULT_COMMAND payload.
    bool apply_local_fault(const uint8_t *payload);

    // JBS link
    void jbs_tick(void);   // resend / timeout
    void send_jbf1(const uint8_t *fault_payload, uint16_t fault_len, uint32_t token, uint8_t source);
    void send_jbs_frame(const char *magic, const uint8_t *payload, uint16_t payload_len);
    void handle_jba1(const uint8_t *payload, uint16_t payload_len);
    void jbs_ack_complete(uint8_t result, uint16_t error_code, uint16_t command_id, uint8_t ack_seq);
    void send_actuator_output(void);   // JBO1: FC -> model servo/throttle PWM

    // senders
    void send_frame(uint8_t msg_id, const uint8_t *payload, uint16_t payload_len, uint8_t flags);
    void send_ack(uint8_t ack_seq, uint8_t ack_msg_id, uint8_t result, uint16_t command_id, uint16_t error_code);
    void send_aircraft_state(void);

    // state assembly
    void build_aircraft_state(uint8_t *payload) const;

    // helpers
    static uint16_t crc16_ccitt(const uint8_t *buf, uint16_t len);
    static uint16_t crc16_ccitt_init(const uint8_t *buf, uint16_t len, uint16_t crc);
    uint8_t src_id(void) const;
};

namespace AP {
    AP_AllSpark *allspark(void);
};

#endif  // AP_ALLSPARK_ENABLED
