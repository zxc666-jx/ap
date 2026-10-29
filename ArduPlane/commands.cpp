/*
 *  logic for dealing with the current command in the mission and home location
 */

#include "Plane.h"

#if AP_EXTERNAL_AHRS_ENABLED
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#endif

/*
 *  set_next_WP - sets the target location the vehicle should fly to
 */
void Plane::set_next_WP(const Location &loc)
{
    if (auto_state.next_wp_crosstrack) {
        // copy the current WP into the OldWP slot
        prev_WP_loc = next_WP_loc;
        auto_state.crosstrack = true;
    } else {
        // we should not try to cross-track for this waypoint
        prev_WP_loc = current_loc;
        // use cross-track for the next waypoint
        auto_state.next_wp_crosstrack = true;
        auto_state.crosstrack = false;
    }

    // Load the next_WP slot
    // ---------------------
    next_WP_loc = loc;

    // if lat and lon is zero, then use current lat/lon
    // this allows a mission to contain a "loiter on the spot"
    // command
    if (next_WP_loc.lat == 0 && next_WP_loc.lng == 0) {
        next_WP_loc.lat = current_loc.lat;
        next_WP_loc.lng = current_loc.lng;
        // additionally treat zero altitude as current altitude
        if (next_WP_loc.alt_is_zero()) {
            next_WP_loc.copy_alt_from(current_loc);
        }
    }

    fix_terrain_WP(next_WP_loc, __AP_LINE__);

    // convert relative alt to absolute alt
    if (!next_WP_loc.terrain_alt) {
        next_WP_loc.change_alt_frame(Location::AltFrame::ABSOLUTE);
    }

    // are we already past the waypoint? This happens when we jump
    // waypoints, and it can cause us to skip a waypoint. If we are
    // past the waypoint when we start on a leg, then use the current
    // location as the previous waypoint, to prevent immediately
    // considering the waypoint complete
    if (current_loc.past_interval_finish_line(prev_WP_loc, next_WP_loc)) {
        prev_WP_loc = current_loc;
    }

    // zero out our loiter vals to watch for missed waypoints
    loiter_angle_reset();

    setup_alt_slope();
    setup_turn_angle();

    // update plane.target_altitude straight away, or if we are too
    // close to out loiter point we may decide we are at the correct
    // altitude before updating it (this is based on scheduler table
    // ordering, where we navigate() before we
    // adjust_altitude_target(), and navigate() uses values updated in
    // adjust_altitude_target()
    adjust_altitude_target();
}

void Plane::set_guided_WP(const Location &loc)
{
    if (aparm.loiter_radius < 0 || loc.loiter_ccw) {
        loiter.direction = -1;
    } else {
        loiter.direction = 1;
    }

    // copy the current location into the OldWP slot
    // ---------------------------------------
    prev_WP_loc = current_loc;

    // Load the next_WP slot
    // ---------------------
    next_WP_loc = loc;

    fix_terrain_WP(next_WP_loc, __AP_LINE__);

    // used to control FBW and limit the rate of climb
    // -----------------------------------------------
    set_target_altitude_current();

    setup_alt_slope();
    setup_turn_angle();

    // disable crosstrack, head directly to the point
    auto_state.crosstrack = false;

    // reset loiter start time.
    loiter.start_time_ms = 0;

    // start in non-VTOL mode
    auto_state.vtol_loiter = false;
    
    loiter_angle_reset();

#if HAL_QUADPLANE_ENABLED
    // cancel pending takeoff
    quadplane.guided_takeoff = false;
#endif
}

// apply an AllSpark guidance command (protocol document section 5).
//
// The FC only applies guidance while it is already in GUIDED - it never changes
// mode or arms itself because of an AllSpark frame. Within GUIDED the command
// maps onto the existing offboard-guided seams:
//   lat/lon/alt -> guided waypoint
//   airspeed    -> guided target airspeed
//   roll        -> external roll target  (guided_state.forced_rpy_cd.x)
//   climb angle -> external pitch target (guided_state.forced_rpy_cd.y), using
//                  target pitch = climb angle + angle of attack
AllSparkGuidanceResult Plane::set_allspark_guidance(const AllSparkGuidance &cmd)
{
    if (control_mode != &mode_guided) {
        return AllSparkGuidanceResult::NOT_GUIDED;
    }

    // Validate everything before touching any state. set_guided_WP() cannot be
    // rolled back, so a command that is going to be rejected must be rejected
    // before the waypoint moves.
    if (cmd.airspeed_mps >= 0.0f &&
        (cmd.airspeed_mps > aparm.airspeed_max || cmd.airspeed_mps < aparm.airspeed_min)) {
        return AllSparkGuidanceResult::OUT_OF_RANGE;
    }

    if (cmd.have_position) {
        Location loc = cmd.loc;
        if (!cmd.have_altitude) {
            // no altitude commanded: hold the current altitude
            loc.set_alt_cm(current_loc.alt, Location::AltFrame::ABOVE_HOME);
        }
        set_guided_WP(loc);
    }

    if (cmd.airspeed_mps >= 0.0f) {
        // the envelope was checked above
        mode_guided.handle_change_airspeed(cmd.airspeed_mps, 0.1f);
    }

    // millis() is zero only in the first tick after boot; keep the timestamp
    // non-zero because zero is what disables the forced-rpy override
    const uint32_t now_ms = MAX(millis(), 1U);

    if (cmd.have_roll) {
        guided_state.forced_rpy_cd.x = (int32_t)(cmd.roll_deg * 100.0f);
        guided_state.last_forced_rpy_ms.x = now_ms;
    }

    if (cmd.have_climb) {
        float aoa_deg = 0.0f;
#if AP_EXTERNAL_AHRS_ENABLED
        const AP_ExternalAHRS::JSBSimTruth &truth = AP::externalAHRS().get_jsbsim_truth();
        if (truth.valid) {
            aoa_deg = truth.aoa_deg;
        }
#endif
        guided_state.forced_rpy_cd.y = (int32_t)((cmd.climb_deg + aoa_deg) * 100.0f);
        guided_state.last_forced_rpy_ms.y = now_ms;
    }

    return AllSparkGuidanceResult::OK;
}

// drop the AllSpark targets after a guidance timeout. The flight mode is left
// to the operator: an RC mode switch may be holding GUIDED, and reverting it
// here would just fight the switch.
void Plane::clear_allspark_guidance(void)
{
    guided_state.last_forced_rpy_ms.x = 0;
    guided_state.last_forced_rpy_ms.y = 0;
    guided_state.target_airspeed_cm = -1;
}

// retrieve the current navigation target (next waypoint / guided WP)
bool Plane::get_allspark_target_location(Location &loc) const
{
    loc = next_WP_loc;
    return true;
}

// scheduler task: AllSpark parsing and command handling run on the main thread
// so that they share the navigation state with the flight loop
void Plane::allspark_update(void)
{
    AP_AllSpark *link = AP::allspark();
    if (link != nullptr) {
        link->update();
    }
}

// periodic fault-injection engine update. The effects are applied where they
// belong: sensor faults in AP_Airspeed/AP_Baro, and throttle/surface faults in
// apply_fault_injection_output(), which runs at the end of set_servos().
void Plane::fault_injection_tick(void)
{
    AP_FaultInjection *fij = AP::fault_injection();
    if (fij == nullptr) {
        return;
    }
    fij->update();
}

// Apply the actuator-side faults. This must run after the control loop has
// produced the servo outputs and before the mixer turns them into PWM,
// otherwise the next loop iteration would overwrite the faulted values.
void Plane::apply_fault_injection_output(void)
{
    AP_FaultInjection *fij = AP::fault_injection();
    if (fij == nullptr || !fij->any_active()) {
        return;
    }

    const struct {
        SRV_Channel::Function func;
        AP_FaultInjection::Surface surf;
    } surfaces[] = {
        { SRV_Channel::k_aileron,  AP_FaultInjection::Surface::AILERON },
        { SRV_Channel::k_elevator, AP_FaultInjection::Surface::ELEVATOR },
        { SRV_Channel::k_rudder,   AP_FaultInjection::Surface::RUDDER },
    };

    // the protocol commands a jam angle in degrees; the airframe's full-scale
    // deflection is not known to the FC, so 45 deg is used as the reference
    // that maps onto the scaled (-100..100) servo command
    static constexpr float SURFACE_FULL_DEFLECTION_DEG = 45.0f;

    for (const auto &s : surfaces) {
        const float jam_deg = fij->get_surface_offset_deg(s.surf);
        if (!is_zero(jam_deg)) {
            // a jam overrides the commanded deflection entirely
            const float jam_scaled = constrain_float(jam_deg / SURFACE_FULL_DEFLECTION_DEG * 100.0f, -100.0f, 100.0f);
            SRV_Channels::set_output_scaled(s.func, jam_scaled);
            continue;
        }
        const float scale = fij->get_surface_scale(s.surf);
        if (scale < 1.0f) {
            SRV_Channels::set_output_scaled(s.func, SRV_Channels::get_output_scaled(s.func) * scale);
        }
    }

    // thrust-loss faults scale the throttle demand
    const float thr_scale = fij->get_throttle_scale();
    if (thr_scale < 1.0f) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle,
                                        SRV_Channels::get_output_scaled(SRV_Channel::k_throttle) * thr_scale);
    }
}

/*
  update home location from GPS
  this is called as long as we have 3D lock and the arming switch is
  not pushed

  returns true if home is changed
*/
bool Plane::update_home()
{
    if (hal.util->was_watchdog_armed()) {
        return false;
    }
    if ((g2.home_reset_threshold == -1) ||
        ((g2.home_reset_threshold > 0) &&
         (fabsf(barometer.get_altitude()) > g2.home_reset_threshold))) {
        // don't auto-update if we have changed barometer altitude
        // significantly. This allows us to cope with slow baro drift
        // but not re-do home and the baro if we have changed height
        // significantly
        return false;
    }
    bool ret = false;
    if (ahrs.home_is_set() && !ahrs.home_is_locked() && gps.status() >= AP_GPS_FixType::FIX_3D) {
        Location loc;
        if (ahrs.get_location(loc)) {
            // we take the altitude directly from the GPS as we are
            // about to reset the baro calibration. We can't use AHRS
            // altitude or we can end up perpetuating a bias in
            // altitude, as AHRS alt depends on home alt, which means
            // we would have a circular dependency
            loc.set_alt_cm(gps.location().alt,
                           Location::AltFrame::ABSOLUTE);
            ret = AP::ahrs().set_home(loc);
        }
    }

    // even if home is not updated we do a baro reset to stop baro
    // drift errors while disarmed
    barometer.update_calibration();
    ahrs.resetHeightDatum();

    update_current_loc();

    return ret;
}

bool Plane::set_home_persistently(const Location &loc)
{
    if (hal.util->was_watchdog_armed()) {
        return false;
    }
    if (!AP::ahrs().set_home(loc)) {
        return false;
    }

    return true;
}
