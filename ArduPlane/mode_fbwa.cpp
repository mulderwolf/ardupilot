#include "mode.h"
#include "Plane.h"

float prv_baro_alt = 250;
float prv_direction = 0;
int8_t rc_err = 0;

void ModeFBWA::update()
{
    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd  = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();
    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max*100;
    } else {
        plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min*100);
    }
    plane.adjust_nav_pitch_throttle();
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min*100, plane.aparm.pitch_limit_max.get()*100);
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }

	if (!(plane.failsafe.rc_failsafe)) {
        rc_err = 0;
        prv_baro_alt = plane.barometer.get_altitude();
        prv_direction = degrees(ahrs.get_yaw())+180;
        if (prv_direction > 360)         
        { 
            prv_direction = prv_direction - 360;
        }
    }

    if (plane.failsafe.rc_failsafe) {
        if (rc_err == 0){
            gcs().send_text(MAV_SEVERITY_INFO, "Return to heading = %i", (int)prv_direction);
            rc_err = 1;
        }
        if (degrees(ahrs.get_yaw()) > prv_direction-5 and degrees(ahrs.get_yaw()) < prv_direction + 5) {
            plane.nav_roll_cd = 0;
        }
        else
        {
            plane.nav_roll_cd = plane.roll_limit_cd / 3;
        }

        if (plane.barometer.get_altitude() > prv_baro_alt){
            plane.nav_pitch_cd = 2;
        }
        else
        {
            plane.nav_pitch_cd = 10;
        }
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 70.0);
    }
	
    RC_Channel *chan = rc().find_channel_for_option(RC_Channel::AUX_FUNC::FBWA_TAILDRAGGER);
    if (chan != nullptr) {
        // check for the user enabling FBWA taildrag takeoff mode
        bool tdrag_mode = chan->get_aux_switch_pos() == RC_Channel::AuxSwitchPos::HIGH;
        if (tdrag_mode && !plane.auto_state.fbwa_tdrag_takeoff_mode) {
            if (plane.auto_state.highest_airspeed < plane.g.takeoff_tdrag_speed1) {
                plane.auto_state.fbwa_tdrag_takeoff_mode = true;
                plane.gcs().send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
            }
        }
    }
}
