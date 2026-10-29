/*
  AP_FaultInjection implementation.
 */
#include "AP_FaultInjection.h"

#if AP_FAULT_INJECTION_ENABLED

extern const AP_HAL::HAL& hal;

AP_FaultInjection *AP_FaultInjection::singleton = nullptr;

AP_FaultInjection::AP_FaultInjection()
{
    singleton = this;
}

namespace AP {
    AP_FaultInjection *fault_injection(void)
    {
        return AP_FaultInjection::get_singleton();
    }
}

void AP_FaultInjection::init(void)
{
    release_all();
    _rng_state = AP_HAL::millis() | 1;
    _last_update_ms = AP_HAL::millis();
    _dt_ms = 0;
    _frozen_airspeed_valid = false;
    _frozen_altitude_valid = false;
    _frozen_pressure_valid = false;
}

// deterministic gaussian-ish noise: sum of 3 uniform in [-0.5,0.5] scaled to std_dev
float AP_FaultInjection::random_noise(float std_dev)
{
    if (std_dev <= 0.0f) {
        return 0.0f;
    }
    float sum = 0.0f;
    for (uint8_t i = 0; i < 3; i++) {
        // xorshift32
        _rng_state ^= _rng_state << 13;
        _rng_state ^= _rng_state >> 17;
        _rng_state ^= _rng_state << 5;
        const float u = (_rng_state & 0xFFFFFF) / 16777215.0f;   // 0..1
        sum += (u - 0.5f);
    }
    // sum of 3 uniform[-0.5,0.5] has std dev ~0.5, scale to the requested std_dev
    return sum * (std_dev * 2.0f);
}

bool AP_FaultInjection::set_fault(uint8_t slot, Type type, uint8_t location,
                                  float p1, float p2,
                                  uint32_t ramp_in_ms, uint32_t ramp_out_ms,
                                  uint32_t start_delay_ms, uint32_t duration_ms)
{
    if (slot >= NUM_SLOTS) {
        return false;
    }
    if (uint8_t(type) > TYPE_MAX || location > LOCATION_MAX) {
        return false;
    }
    // time bounds per protocol (max 3600 s)
    if (ramp_in_ms > MAX_TIME_MS || ramp_out_ms > MAX_TIME_MS ||
        start_delay_ms > MAX_TIME_MS || duration_ms > MAX_TIME_MS) {
        return false;
    }
    // ramp-in only meaningful for ramp types (2/5)
    if (ramp_in_ms != 0 && !(type == Type::THRUST_LOSS_RAMP || type == Type::SURFACE_EFF_RAMP)) {
        return false;
    }
    // surface faults must name a surface
    if ((type == Type::SURFACE_JAM || type == Type::SURFACE_EFF_STEP || type == Type::SURFACE_EFF_RAMP) &&
        (location < uint8_t(Surface::ELEVATOR) || location > uint8_t(Surface::AILERON))) {
        return false;
    }

    FaultState &f = _slots[slot];
    f.type = type;
    f.location = location;
    f.p1 = p1;
    f.p2 = p2;
    f.ramp_in_ms = ramp_in_ms;
    f.ramp_out_ms = ramp_out_ms;
    f.start_delay_ms = start_delay_ms;
    f.duration_ms = duration_ms;

    f.armed = true;
    f.active = false;
    f.trigger_ms = AP_HAL::millis();
    f.ramp_progress = 0.0f;
    f.current_severity = 0.0f;
    f.jam_value = p1;          // jam angle = P1 (deg)
    f.drift_value = 0.0f;
    f.noise_value = 0.0f;

    // a new fault on this slot invalidates any latched freeze value
    _frozen_airspeed_valid = false;
    _frozen_altitude_valid = false;
    _frozen_pressure_valid = false;
    return true;
}

void AP_FaultInjection::release_slot(uint8_t slot)
{
    if (slot < NUM_SLOTS) {
        _slots[slot].armed = false;
        _slots[slot].active = false;
        _slots[slot].type = Type::NONE;
        _frozen_airspeed_valid = false;
        _frozen_altitude_valid = false;
        _frozen_pressure_valid = false;
    }
}

void AP_FaultInjection::release_all(void)
{
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        release_slot(i);
    }
}

uint8_t AP_FaultInjection::find_free_slot(void) const
{
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        if (!_slots[i].armed) {
            return i;
        }
    }
    return NUM_SLOTS;
}

uint8_t AP_FaultInjection::active_count(void) const
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        if (_slots[i].armed) {
            count++;
        }
    }
    return count;
}

void AP_FaultInjection::update(uint32_t dt_ms)
{
    const uint32_t now = AP_HAL::millis();
    if (dt_ms == 0) {
        dt_ms = now - _last_update_ms;
    }
    _dt_ms = dt_ms;
    _last_update_ms = now;

    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        recalc_slot(i);
    }
}

void AP_FaultInjection::recalc_slot(uint8_t slot)
{
    FaultState &f = _slots[slot];
    if (!f.armed) {
        return;
    }

    const uint32_t now = AP_HAL::millis();
    const uint32_t elapsed = now - f.trigger_ms;

    // before start delay: not applied
    if (elapsed < f.start_delay_ms) {
        f.active = false;
        f.current_severity = 0.0f;
        return;
    }

    f.active = true;

    // envelope severity 0..1
    float sev = 1.0f;
    const uint32_t t_apply = elapsed - f.start_delay_ms;

    if (f.ramp_in_ms != 0 && t_apply < f.ramp_in_ms) {
        sev = (float)t_apply / (float)f.ramp_in_ms;
    } else if (f.duration_ms != 0) {
        const uint32_t t_hold_end = f.ramp_in_ms + f.duration_ms;
        if (t_apply >= t_hold_end) {
            if (f.ramp_out_ms != 0 && t_apply < t_hold_end + f.ramp_out_ms) {
                sev = 1.0f - (float)(t_apply - t_hold_end) / (float)f.ramp_out_ms;
            } else {
                // ended
                f.armed = false;
                f.active = false;
                f.current_severity = 0.0f;
                return;
            }
        }
    }
    f.ramp_progress = sev;
    f.current_severity = sev;

    // per-type runtime state
    switch (f.type) {
    case Type::AIRSPEED_NOISE:
    case Type::ALTITUDE_NOISE:
        f.noise_value = random_noise(f.p1);
        break;
    case Type::AIRSPEED_DRIFT:
    case Type::ALTITUDE_DRIFT:
        f.drift_value += f.p1 * (_dt_ms * 0.001f);
        f.drift_value = constrain_float(f.drift_value, -fabsf(f.p2), fabsf(f.p2));
        break;
    default:
        break;
    }
}

float AP_FaultInjection::get_throttle_scale(void) const
{
    float scale = 1.0f;
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        const FaultState &f = _slots[i];
        if (!f.active) {
            continue;
        }
        if (f.type == Type::THRUST_LOSS_STEP || f.type == Type::THRUST_LOSS_RAMP) {
            scale *= (1.0f - f.current_severity * constrain_float(f.p1, 0.0f, 1.0f));
        }
    }
    return constrain_float(scale, 0.0f, 1.0f);
}

float AP_FaultInjection::get_surface_scale(Surface surf) const
{
    float scale = 1.0f;
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        const FaultState &f = _slots[i];
        if (!f.active) {
            continue;
        }
        if (f.location != (uint8_t)surf) {
            continue;
        }
        if (f.type == Type::SURFACE_EFF_STEP || f.type == Type::SURFACE_EFF_RAMP) {
            scale *= (1.0f - f.current_severity * constrain_float(f.p1, 0.0f, 1.0f));
        }
    }
    return constrain_float(scale, 0.0f, 1.0f);
}

float AP_FaultInjection::get_surface_offset_deg(Surface surf) const
{
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        const FaultState &f = _slots[i];
        if (!f.active) {
            continue;
        }
        if (f.location != (uint8_t)surf) {
            continue;
        }
        if (f.type == Type::SURFACE_JAM) {
            return f.jam_value * f.current_severity;
        }
    }
    return 0.0f;
}

float AP_FaultInjection::get_airspeed_modifier(void) const
{
    float mod = 0.0f;
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        const FaultState &f = _slots[i];
        if (!f.active) {
            continue;
        }
        switch (f.type) {
        case Type::AIRSPEED_NOISE:
            mod += f.noise_value;
            break;
        case Type::AIRSPEED_DRIFT:
            mod += f.drift_value;
            break;
        case Type::AIRSPEED_BIAS:
            mod += f.p1;
            break;
        default:
            break;
        }
    }
    return mod;
}

float AP_FaultInjection::get_altitude_modifier(void) const
{
    float mod = 0.0f;
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        const FaultState &f = _slots[i];
        if (!f.active) {
            continue;
        }
        switch (f.type) {
        case Type::ALTITUDE_NOISE:
            mod += f.noise_value;
            break;
        case Type::ALTITUDE_DRIFT:
            mod += f.drift_value;
            break;
        case Type::ALTITUDE_BIAS:
            mod += f.p1;
            break;
        default:
            break;
        }
    }
    return mod;
}

bool AP_FaultInjection::airspeed_frozen(void) const
{
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        if (_slots[i].active && _slots[i].type == Type::AIRSPEED_FREEZE) {
            return true;
        }
    }
    return false;
}

bool AP_FaultInjection::altitude_frozen(void) const
{
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        if (_slots[i].active && _slots[i].type == Type::ALTITUDE_FREEZE) {
            return true;
        }
    }
    return false;
}

bool AP_FaultInjection::airspeed_fault_active(void) const
{
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        const FaultState &f = _slots[i];
        if (!f.active) {
            continue;
        }
        switch (f.type) {
        case Type::AIRSPEED_NOISE:
        case Type::AIRSPEED_DRIFT:
        case Type::AIRSPEED_BIAS:
        case Type::AIRSPEED_FREEZE:
            return true;
        default:
            break;
        }
    }
    return false;
}

bool AP_FaultInjection::altitude_fault_active(void) const
{
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        const FaultState &f = _slots[i];
        if (!f.active) {
            continue;
        }
        switch (f.type) {
        case Type::ALTITUDE_NOISE:
        case Type::ALTITUDE_DRIFT:
        case Type::ALTITUDE_BIAS:
        case Type::ALTITUDE_FREEZE:
            return true;
        default:
            break;
        }
    }
    return false;
}

// Airspeed faults are applied at the sensor driver, so the fault propagates to
// every consumer (TECS, EKF airspeed fusion, arming checks, telemetry).
float AP_FaultInjection::apply_airspeed(float airspeed) const
{
    if (airspeed < 0.0f || !airspeed_fault_active()) {
        return airspeed;
    }
    float out = airspeed + get_airspeed_modifier();
    if (airspeed_frozen()) {
        if (!_frozen_airspeed_valid) {
            _frozen_airspeed = out;
            _frozen_airspeed_valid = true;
        }
        out = _frozen_airspeed;
    }
    return MAX(out, 0.0f);
}

// Altitude faults are applied to the barometer. EKF3 reads baro altitude
// directly (AP_NavEKF3_Measurements), so biasing here corrupts the altitude
// estimate that every controller uses.
float AP_FaultInjection::apply_altitude(float altitude) const
{
    if (!altitude_fault_active()) {
        return altitude;
    }
    float out = altitude + get_altitude_modifier();
    if (altitude_frozen()) {
        if (!_frozen_altitude_valid) {
            _frozen_altitude = out;
            _frozen_altitude_valid = true;
        }
        out = _frozen_altitude;
    }
    return out;
}

// Same fault expressed in the pressure domain, so consumers that read pressure
// (rather than altitude) see a consistent error. The linearisation
// dp/dh = -p / 8434 m is the standard-atmosphere scale height near sea level.
float AP_FaultInjection::apply_pressure(float pressure) const
{
    if (pressure <= 0.0f || !altitude_fault_active()) {
        return pressure;
    }
    float out = pressure * (1.0f - get_altitude_modifier() / 8434.0f);
    if (altitude_frozen()) {
        if (!_frozen_pressure_valid) {
            _frozen_pressure = out;
            _frozen_pressure_valid = true;
        }
        out = _frozen_pressure;
    }
    return MAX(out, 1.0f);
}

#endif  // AP_FAULT_INJECTION_ENABLED
