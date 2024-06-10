#include "mode.h"
#include "Plane.h"

int32_t prv_nav_roll_cd = 0;
int32_t prv_nav_pitch_cd = 0;
float prv_throttle = 50.0;

bool ModeCMBT::_enter()
{
    gcs().send_text(MAV_SEVERITY_WARNING, "Enter CombatMode");
    return true;
}

void ModeCMBT::update()
{
    // set nav_roll and nav_pitch using sticks
    plane.nav_roll_cd = plane.channel_roll->norm_input() * plane.roll_limit_cd;
    plane.update_load_factor();

    float pitch_input = plane.channel_pitch->norm_input();
    if (pitch_input > 0) {
        plane.nav_pitch_cd = pitch_input * plane.aparm.pitch_limit_max * 100;
    }
    else {
        plane.nav_pitch_cd = -(pitch_input * plane.pitch_limit_min * 100);
    }
    plane.nav_pitch_cd = constrain_int32(plane.nav_pitch_cd, plane.pitch_limit_min * 100, plane.aparm.pitch_limit_max.get() * 100);
    if (plane.fly_inverted()) {
        plane.nav_pitch_cd = -plane.nav_pitch_cd;
    }
    const float throttle = plane.get_throttle_input(true);
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, throttle);


    if (!(plane.failsafe.rc_failsafe)) {
        //prv_nav_roll_cd = plane.nav_roll_cd;
        //prv_nav_pitch_cd = plane.nav_pitch_cd;
        //prv_throttle = throttle;
        //gcs().send_text(MAV_SEVERITY_INFO, "Switch to FBWA");
        plane.set_mode(plane.mode_acro, ModeReason::MISSION_CMD);
    }

    //if (millis() - plane.failsafe.last_valid_rc_ms > 100) {}
    if (plane.failsafe.rc_failsafe) {
        plane.nav_roll_cd = prv_nav_roll_cd;
        plane.nav_pitch_cd = prv_nav_pitch_cd;
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, prv_throttle);
        plane.set_mode(plane.mode_cmbt, ModeReason::FAILSAFE);
    }
}
