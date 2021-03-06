/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AC_PRECLAND_BACKEND_H__
#define __AC_PRECLAND_BACKEND_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_PID.h>
#include <AP_InertialNav/AP_InertialNav.h>
#include "AC_PrecLand.h"

class AC_PrecLand_Backend
{
public:

    // Constructor
    AC_PrecLand_Backend(const AC_PrecLand& frontend, AC_PrecLand::precland_state& state) :
        _frontend(frontend),
        _state(state) {}

    // destructor
    virtual ~AC_PrecLand_Backend() {}

    // init - perform any required initialisation of backend controller
    virtual void init() = 0;

    // update - give chance to driver to get updates from sensor
    //  returns true if new data available
    virtual bool update() = 0;
    // what frame of reference is our sensor reporting in?
    virtual MAV_FRAME get_frame_of_reference() = 0;
	
	// returns expected time period in seconds between samples
    virtual float get_delta_time() const = 0;

    // get_angle_to_target - returns angles (in radians) to target
    //  returns true if angles are available, false if not (i.e. no target)
    //  x_angle_rad : roll direction, positive = target is to right (looking down)
    //  y_angle_rad : pitch direction, positive = target is forward (looking down)
    //  size_rad : target's size in radians
    //  capture_time_ms : system time in milliseconds that angles were captured
    virtual bool get_angle_to_target(float &x_angle_rad, float &y_angle_rad, float &size_rad, uint32_t &capture_time_ms) = 0;

    // handle_msg - parses a mavlink message from the companion computer
    virtual void handle_msg(mavlink_message_t* msg) = 0;

protected:

    const AC_PrecLand&  _frontend;          // reference to precision landing front end
    AC_PrecLand::precland_state &_state;    // reference to this instances state
};


#endif	// __AC_PRECLAND_BACKEND_H__