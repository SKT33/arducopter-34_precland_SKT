/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL/AP_HAL.h>
#include "AC_PrecLand.h"
#include "AC_PrecLand_Backend.h"
#include "AC_PrecLand_Companion.h"
#include "AC_PrecLand_IRLock.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_PrecLand::var_info[] = {
    // @Param: ENABLED
    // @DisplayName: Precision Land enabled/disabled and behaviour
    // @Description: Precision Land enabled/disabled and behaviour
    // @Values: 0:Disabled, 1:Enabled Always Land, 2:Enabled Strict
    // @User: Advanced
    AP_GROUPINFO("ENABLED", 0, AC_PrecLand, _enabled, 0),

    // @Param: TYPE
    // @DisplayName: Precision Land Type
    // @Description: Precision Land Type
    // @Values: 0:None, 1:CompanionComputer, 2:IRLock
    // @User: Advanced
    AP_GROUPINFO("TYPE",    1, AC_PrecLand, _type, 0),
	
	// @Param: SPEED
    // @DisplayName: Precision Land horizontal speed maximum in cm/s
    // @Description: Precision Land horizontal speed maximum in cm/s
    // @Range: 0 500
    // @User: Advanced
    AP_GROUPINFO("SPEED",   2, AC_PrecLand, _speed_xy, AC_PRECLAND_SPEED_XY_DEFAULT),

    // @Param: SIZE_RAD
    // @DisplayName: Precision Landing target's apparent size at 1m distance in radians
    // @Description: Precision Landing target's apparent size at 1m distance in radians
    // @Range: 0.01 1.5
    // @Values: 10cm:0.1, 15cm:0.15, 20cm:0.2
    // @User: Advanced
    AP_GROUPINFO("TARG_SIZE",   3, AC_PrecLand, _target_size_1m_rad, PRECLAND_TARGET_AT_1M_SIZE_RAD_DEFAULT),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_PrecLand::AC_PrecLand(const AP_AHRS& ahrs, const AP_InertialNav& inav,
                         AC_PI_2D& pi_precland_xy) :
    _ahrs(ahrs),
    _inav(inav),
    _pi_precland_xy(pi_precland_xy),
    _size_rad(0.0f),
    _size_rad_filter(PRECLAND_TARGET_SIZE_FILT_HZ),
    _distance_est(0.0f),
    _capture_time_ms(0),
    _have_estimate(false),
    _limit_xy(false),
    _desired_vel_filter(PRECLAND_DESVEL_FILTER_HZ),
    _buff_ahrs_sin_roll(),
    _buff_ahrs_cos_roll(),
    _buff_ahrs_sin_pitch(),
    _buff_ahrs_cos_pitch(),
    _buff_ahrs_sin_yaw(),
    _buff_ahrs_cos_yaw(),
    _backend(NULL)
{
    // set parameters to defaults
    AP_Param::setup_object_defaults(this, var_info);

    // other initialisation
    _backend_state.healthy = false;
	
	// clear buffers
    _buff_ahrs_sin_roll.clear();
    _buff_ahrs_cos_roll.clear();
    _buff_ahrs_sin_pitch.clear();
    _buff_ahrs_cos_pitch.clear();
    _buff_ahrs_sin_yaw.clear();
    _buff_ahrs_cos_yaw.clear();
}


// init - perform any required initialisation of backends
void AC_PrecLand::init()
{
    // exit immediately if init has already been run
    if (_backend != NULL) {
        return;
    }

    // default health to false
    _backend = NULL;
    _backend_state.healthy = false;

    // instantiate backend based on type parameter
    switch ((enum PrecLandType)(_type.get())) {
        // no type defined
        case PRECLAND_TYPE_NONE:
        default:
            return;
        // companion computer
        case PRECLAND_TYPE_COMPANION:
            _backend = new AC_PrecLand_Companion(*this, _backend_state);
            break;
        // IR Lock
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
        case PRECLAND_TYPE_IRLOCK:
            _backend = new AC_PrecLand_IRLock(*this, _backend_state);
            break;
#endif
    }

    // init backend
    if (_backend != NULL) {
        _backend->init();
		pi_precland_xy.set_dt(_backend->get_delta_time());
    }
}

// update - give chance to driver to get updates from sensor
bool AC_PrecLand::update(float alt_above_terrain_cm)
{
    bool updated = false;

    // run backend update
    if (_backend != NULL) {
        // read from sensor
        updated = _backend->update();

        // calculate angles to target
        calc_angles(alt_above_terrain_cm);

        // update attitude buffers
        if (updated) {
            _buff_ahrs_sin_roll.push_back(_ahrs.sin_roll());
            _buff_ahrs_cos_roll.push_back(_ahrs.cos_roll());
            _buff_ahrs_sin_pitch.push_back(_ahrs.sin_pitch());
            _buff_ahrs_cos_pitch.push_back(_ahrs.cos_pitch());
            _buff_ahrs_sin_yaw.push_back(_ahrs.sin_yaw());
            _buff_ahrs_cos_yaw.push_back(_ahrs.cos_yaw());
        }
    }

    return updated;
}

// initialise desired velocity
void AC_PrecLand::set_desired_velocity(const Vector3f &des_vel)
{
    _desired_vel = des_vel;
    _desired_vel_filter.reset(Vector3f(0.0f,0.0f,0.0f));
    _pi_precland_xy.reset_filter();
    _pi_precland_xy.set_integrator(Vector2f(des_vel.x/100.0f,des_vel.y/100.0f));
}

// get target 3D velocity towards target
const Vector3f& AC_PrecLand::calc_desired_velocity(float land_speed_cms)
{
    // return zero velocity if not enabled
    if (_backend == NULL) {
        _desired_vel.zero();
        return _desired_vel;
    }

    // ensure land_speed_cms is positive
    land_speed_cms = fabsf(land_speed_cms);

    // increase gain with distance (i.e. gain is 0.1 at 1m, 1.0 at 10m)
    float gain = constrain_float(_distance_est, PRECLAND_VELGAIN_DISTANCE_MIN, PRECLAND_VELGAIN_DISTANCE_MAX) / PRECLAND_VELGAIN_DISTANCE_MAX;

    // set velocity to simply land-speed if last sensor update more than 1 second ago
    uint32_t dt = hal.scheduler->millis() - _capture_time_ms;
    if (dt > PRECLAND_SENSOR_TIMEOUT_MS) {
        _desired_vel.x = 0.0f;
        _desired_vel.y = 0.0f;
        _desired_vel.z = -land_speed_cms;
        _pi_precland_xy.reset_I();
        _pi_precland_xy.reset_filter();
        _limit_xy = false;
        _size_rad_reset = true;

    } else if (_have_estimate) {

        _desired_vel.x = _vec_to_target_ef.x * _pi_precland_xy.kP() * gain;
        _desired_vel.y = _vec_to_target_ef.y * _pi_precland_xy.kP() * gain;
        //_desired_vel.z = -1.0f;
        //_desired_vel.normalize();
        //_desired_vel.z = min(0.0f, (-1.0f+(pythagorous2(_vec_to_target_ef.x, _vec_to_target_ef.y)*PRECLAND_CAUTION_GAIN)));
        _desired_vel.z = min(0.0f, (-1.0f+(pythagorous2(_vec_to_target_ef.x, _vec_to_target_ef.y)*_pi_precland_xy.kI())));
        _desired_vel *= land_speed_cms;
    }

    // record we have consumed any reading
    _have_estimate = false;

    // filter output
    _desired_vel_filter.apply(_desired_vel, 0.0025f);

    // return desired velocity
    return _desired_vel_filter.get();
}

// calc_angles - converts sensor's body-frame angles to earth-frame angles and desired velocity
//  raw sensor angles stored in _angle_to_target (might be in earth frame, or maybe body frame)
//  earth-frame angles stored in _ef_angle_to_target
void AC_PrecLand::calc_angles(float alt_above_terrain_cm)
{
    // exit immediately if not enabled
    if (_backend == NULL) {
        _have_estimate = false;
        return;
    }

    // get angles to target from backend
    if (!_backend->get_angle_to_target(_angle_to_target.x, _angle_to_target.y, _size_rad, _capture_time_ms)) {
        _have_estimate = false;
        return;
    }

    if(_backend->get_frame_of_reference() == MAV_FRAME_BODY_NED) {
        // angles provided in body frame
        Vector3f vec_to_target_bf(sinf(-_angle_to_target.y), sinf(_angle_to_target.x), 1.0f);
        if (!is_zero(_ahrs.cos_pitch())) {
            // convert earth frame vector angle to body frame
            float sin_roll = _buff_ahrs_sin_roll.peek(0);
            float cos_roll = _buff_ahrs_cos_roll.peek(0);
            float sin_pitch = _buff_ahrs_sin_pitch.peek(0);
            float cos_pitch = _buff_ahrs_cos_pitch.peek(0);
            float sin_yaw = _buff_ahrs_sin_yaw.peek(0);
            float cos_yaw = _buff_ahrs_cos_yaw.peek(0);
            _vec_to_target_ef.x = (cos_pitch * cos_yaw) * vec_to_target_bf.x +
                (sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw) * vec_to_target_bf.y +
                (cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw) * vec_to_target_bf.z;

            _vec_to_target_ef.y = (cos_pitch * sin_yaw) * vec_to_target_bf.x +
                (sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw) * vec_to_target_bf.y +
                (cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw) * vec_to_target_bf.z;

            _vec_to_target_ef.z = -sin_pitch * vec_to_target_bf.x +
                sin_roll * cos_pitch * vec_to_target_bf.y +
                cos_roll * cos_pitch * vec_to_target_bf.z;
            /*
            _vec_to_target_ef.x = (_ahrs.cos_pitch() * _ahrs.cos_yaw()) * vec_to_target_bf.x +
                (_ahrs.sin_roll() * _ahrs.sin_pitch() * _ahrs.cos_yaw() - _ahrs.cos_roll() * _ahrs.sin_yaw()) * vec_to_target_bf.y +
                (_ahrs.cos_roll() * _ahrs.sin_pitch() * _ahrs.cos_yaw() + _ahrs.sin_roll() * _ahrs.sin_yaw()) * vec_to_target_bf.z;

            _vec_to_target_ef.y = (_ahrs.cos_pitch() * _ahrs.sin_yaw()) * vec_to_target_bf.x +
                (_ahrs.sin_roll() * _ahrs.sin_pitch() * _ahrs.sin_yaw() + _ahrs.cos_roll() * _ahrs.cos_yaw()) * vec_to_target_bf.y +
                (_ahrs.cos_roll() * _ahrs.sin_pitch() * _ahrs.sin_yaw() - _ahrs.sin_roll() * _ahrs.cos_yaw()) * vec_to_target_bf.z;

            _vec_to_target_ef.z = -_ahrs.sin_pitch() * vec_to_target_bf.x +
                _ahrs.sin_roll() * _ahrs.cos_pitch() * vec_to_target_bf.y +
                _ahrs.cos_roll() * _ahrs.cos_pitch() * vec_to_target_bf.z;
            */

            _vec_to_target_ef.normalize();
        } else {
            _vec_to_target_ef.zero();
        }
    } else {
        // angles already in earth-frame
        _vec_to_target_ef(sinf(-_angle_to_target.y), sinf(_angle_to_target.x), 1.0f);
        _vec_to_target_ef.normalize();

        // rotate to point north
        float x = _vec_to_target_ef.x;
        float y = _vec_to_target_ef.y;
        _vec_to_target_ef.x = x*_ahrs.cos_yaw() - y*_ahrs.sin_yaw();
        _vec_to_target_ef.y = x*_ahrs.sin_yaw() + y*_ahrs.cos_yaw();
    }

    // filter the size
    if (_size_rad_reset) {
        _size_rad_filter.reset(_size_rad);
    } else {
        _size_rad_filter.apply(_size_rad,_backend->get_delta_time());
    }

    // calculate distance estimate
    if (is_zero(_target_size_1m_rad)) {
        // if target size is not specified use altitude above terrain
        _distance_est = alt_above_terrain_cm / 100.0f;
    } else {
        // if target is very small, default distance to very far (100m)
        float size = _size_rad_filter.get();
        if (size <= 0.0f) {
            _distance_est = PRECLAND_DISTANCE_EST_VERY_FAR;
        } else {
            _distance_est = max((_target_size_1m_rad / size),0.0f);
        }
    }

    _have_estimate = true;
}



// handle_msg - Process a LANDING_TARGET mavlink message
void AC_PrecLand::handle_msg(mavlink_message_t* msg)
{
    // run backend update
    if (_backend != NULL) {
        _backend->handle_msg(msg);
    }
}
