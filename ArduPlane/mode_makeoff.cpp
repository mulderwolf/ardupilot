#include "mode.h"
#include "Plane.h"
#include <GCS_MAVLink/GCS.h>

/*
  mode makeoff parameters
 */
ModeTakeoff takeOff;
bool takeoff_mode_setup;
float baro_alt;

bool ModeMakeoff::_enter()
{
    gcs().send_text(MAV_SEVERITY_WARNING, "Makeoff mode");
    takeoff_started = false;
    plane.takeoff_state.accel_event_counter = 0;
    plane.takeoff_state.launchTimerStarted = false;
    plane.takeoff_state.last_tkoff_arm_time = 0;
    return true;
}

void ModeMakeoff::update()
{
    const float alt = takeOff.target_alt;
    uint32_t now = AP_HAL::millis();
    if (!takeoff_mode_setup) {
        const float direction = degrees(ahrs.get_yaw());
        // see if we will skip takeoff as already flying
        if (plane.is_flying() && (millis() - plane.started_flying_ms > 80000U) && ahrs.groundspeed() > 3) {
            takeoff_started = true;
            plane.set_flight_stage(AP_FixedWing::FlightStage::NORMAL);
            // not flying so do a full takeoff sequence
        }
        else {
            // setup makeoff
            plane.crash_state.is_crashed = false;
            plane.auto_state.takeoff_pitch_cd = takeOff.level_pitch * 100;
            plane.auto_state.baro_takeoff_alt = plane.barometer.get_altitude();
            baro_alt = plane.barometer.get_altitude();
            plane.set_flight_stage(AP_FixedWing::FlightStage::TAKEOFF);
            if (!plane.throttle_suppressed) {
                gcs().send_text(MAV_SEVERITY_INFO, "Takeoff to %.0fm heading %.1f deg ,pitch %i",
                    alt, direction, (int)takeOff.level_pitch * 100);
                plane.takeoff_state.start_time_ms = millis();
                takeoff_mode_setup = true;
            }
        }
    }

    if (plane.flight_stage == AP_FixedWing::FlightStage::TAKEOFF) {
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100.0);
        plane.nav_roll_cd = 0;
        plane.takeoff_calc_pitch();
    }
    else {
        if ((plane.barometer.get_altitude() >= baro_alt + alt*100) &&
            (now > plane.takeoff_state.start_time_ms + 15000U)) {
            gcs().send_text(MAV_SEVERITY_INFO, "Switch to FBWA");
            plane.set_mode(plane.mode_fbwa, ModeReason::MISSION_CMD);
            plane.nav_roll_cd = 0;
            plane.nav_pitch_cd = 0;
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 50.0);
        }
        else {
            // still climbing to TAKEOFF_ALT
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 100.0);
            plane.nav_roll_cd = 0;
            plane.takeoff_calc_pitch();
        }
    }
    
}
