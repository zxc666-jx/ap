/*
  AP_FaultInjection - in-flight fault injection for fault-tolerant control research.

  Implements the "Allspark 与 CUAV V5+ 二进制通信协议" fault model (13 types)
  as a flight-controller-internal engine. The engine is driven by a periodic
  update() and exposes per-surface / per-sensor modification getters that the
  vehicle applies inside its own fault_injection_tick().

  The actuator faults are applied by ArduPlane in apply_fault_injection_output()
  and the sensor faults by AP_Airspeed / AP_Baro, so the engine itself stays
  vehicle agnostic.

  Fault types (see protocol document section 6):
    0 none
    1 thrust loss step      P1 = loss ratio 0..1
    2 thrust loss ramp      P1 = loss ratio 0..1
    3 surface jam           P1 = jam angle deg
    4 surface eff step      P1 = loss ratio 0..1
    5 surface eff ramp      P1 = loss ratio 0..1
    6 airspeed noise        P1 = std dev m/s
    7 airspeed drift        P1 = drift rate (m/s)/s, P2 = max deviation m/s
    8 airspeed bias         P1 = bias m/s
    9 airspeed freeze
    10 altitude noise       P1 = std dev m
    11 altitude drift       P1 = drift rate m/s, P2 = max deviation m
    12 altitude bias        P1 = bias m
    13 altitude freeze
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>

#include "AP_FaultInjection_config.h"

#if AP_FAULT_INJECTION_ENABLED

class AP_FaultInjection {
public:
    CLASS_NO_COPY(AP_FaultInjection);

    AP_FaultInjection();

    static AP_FaultInjection *get_singleton(void) { return singleton; }

    // fault type (values match protocol document section 6)
    enum class Type : uint8_t {
        NONE = 0,
        THRUST_LOSS_STEP = 1,
        THRUST_LOSS_RAMP = 2,
        SURFACE_JAM = 3,
        SURFACE_EFF_STEP = 4,
        SURFACE_EFF_RAMP = 5,
        AIRSPEED_NOISE = 6,
        AIRSPEED_DRIFT = 7,
        AIRSPEED_BIAS = 8,
        AIRSPEED_FREEZE = 9,
        ALTITUDE_NOISE = 10,
        ALTITUDE_DRIFT = 11,
        ALTITUDE_BIAS = 12,
        ALTITUDE_FREEZE = 13,
    };

    // surface identifiers for surface faults (fault_location in the protocol)
    enum class Surface : uint8_t {
        VEHICLE = 0,
        ELEVATOR = 1,
        RUDDER = 2,
        AILERON = 3,
        MOTOR = 4,
        AIRSPEED = 5,
        ALTIMETER = 6,
    };

    // highest fault type accepted by set_fault()
    static constexpr uint8_t TYPE_MAX = 13;
    // highest fault location accepted by set_fault()
    static constexpr uint8_t LOCATION_MAX = 6;

    static constexpr uint8_t NUM_SLOTS = 2;   // FC-internal uses 2 slots

    // maximum ramp/delay/duration time (protocol: 3600 s)
    static constexpr uint32_t MAX_TIME_MS = 3600UL * 1000UL;

    struct FaultState {
        Type type = Type::NONE;
        uint8_t location;
        float p1;
        float p2;
        uint32_t ramp_in_ms;
        uint32_t ramp_out_ms;
        uint32_t start_delay_ms;
        uint32_t duration_ms;

        // runtime state
        bool armed = false;
        bool active = false;          // currently applied (past start delay, before end)
        uint32_t trigger_ms;          // time the command was accepted
        float ramp_progress;          // 0..1 ramp envelope
        float current_severity;       // 0..1 current magnitude
        float jam_value;              // surface jam deflection (deg)
        float drift_value;            // accumulated drift
        float noise_value;            // current noise sample
    };

    // initialise (nothing to allocate, provided for symmetry)
    void init(void);

    // periodic update - advance ramp / noise / drift / freeze state.
    // dt_ms is the elapsed time since the previous call.
    void update(uint32_t dt_ms = 0);

    // inject / update / release a fault slot. Returns true on acceptance.
    bool set_fault(uint8_t slot, Type type, uint8_t location,
                   float p1, float p2,
                   uint32_t ramp_in_ms, uint32_t ramp_out_ms,
                   uint32_t start_delay_ms, uint32_t duration_ms);
    void release_slot(uint8_t slot);
    void release_all(void);

    // find a free slot, or NUM_SLOTS if none is available
    uint8_t find_free_slot(void) const;

    // number of currently active (armed) slots
    uint8_t active_count(void) const;
    bool any_active(void) const { return active_count() > 0; }

    // ---- application getters (used by the vehicle fault_injection_tick) ----
    // combined throttle scale 0..1 (thrust-loss faults)
    float get_throttle_scale(void) const;
    // combined surface effectiveness scale 0..1 for a surface
    float get_surface_scale(Surface surf) const;
    // commanded surface offset (deg) from surface-jam faults
    float get_surface_offset_deg(Surface surf) const;
    // airspeed modifier (m/s) to ADD to the measured airspeed
    float get_airspeed_modifier(void) const;
    // altitude modifier (m) to ADD to the measured altitude
    float get_altitude_modifier(void) const;
    // true if airspeed is currently frozen
    bool airspeed_frozen(void) const;
    bool altitude_frozen(void) const;

    // ---- sensor-side application ------------------------------------------
    // These are called from the sensor drivers (AP_Airspeed / AP_Baro) so the
    // fault is seen by everything downstream, including the EKF. They are
    // const so that they can be used from the drivers' const getters; the
    // freeze latches are mutable caches.
    float apply_airspeed(float airspeed) const;
    float apply_altitude(float altitude) const;
    float apply_pressure(float pressure) const;

    // slot introspection (for status/ACK/logging)
    const FaultState &slot(uint8_t i) const { return _slots[i]; }

private:
    static AP_FaultInjection *singleton;

    FaultState _slots[NUM_SLOTS];
    uint32_t _last_update_ms;
    uint32_t _dt_ms;
    uint32_t _rng_state;

    // freeze latches, captured on the first call after a freeze fault becomes
    // active and cleared when it is released
    mutable float _frozen_airspeed;
    mutable float _frozen_altitude;
    mutable float _frozen_pressure;
    mutable bool _frozen_airspeed_valid;
    mutable bool _frozen_altitude_valid;
    mutable bool _frozen_pressure_valid;

    // true if a sensor fault of the given kind is active this instant
    bool airspeed_fault_active(void) const;
    bool altitude_fault_active(void) const;

    float random_noise(float std_dev);
    void recalc_slot(uint8_t slot);
};

namespace AP {
    AP_FaultInjection *fault_injection(void);
};

#endif  // AP_FAULT_INJECTION_ENABLED
