#pragma once

/// @file	AC_P_1D.h
/// @brief	Generic P controller, with EEPROM-backed storage of constants.

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

/// @class	AC_P_1D
/// @brief	Object managing one P controller
class AC_P_1D {
public:

    // constructor
    AC_P_1D(float initial_p, float dt);

    // set time step in seconds
    void set_dt(float dt) { _dt = dt; }

    // update_all - set target and measured inputs to P controller and calculate outputs
    // target and measurement are filtered
    // if measurement is further than error_min or error_max (see set_limits_error method)
    //   the target is moved closer to the measurement and limit_min or limit_max will be set true
    float update_all(float &target, float measurement, bool &limit_min, bool &limit_max) WARN_IF_UNUSED;

    // save gain to eeprom
    void save_gains() { _kp.save(); }

    // set limits on error, output and output from D term
    void set_limits_error(float error_min, float error_max, float output_min, float output_max, float D_Out_max = 0.0f, float D2_Out_max = 0.0f);

    // accessors
    AP_Float &kP() WARN_IF_UNUSED { return _kp; }
    const AP_Float &kP() const WARN_IF_UNUSED { return _kp; }
    void kP(float v) { _kp.set(v); }

    // parameter var table
    static const struct AP_Param::GroupInfo var_info[];

private:

    // parameters
    AP_Float _kp;

    // internal variables
    float _dt;          // time step in seconds
    float _lim_err_min; // error limit in negative direction
    float _lim_err_max; // error limit in positive direction
    float _lim_D_Out;   // maximum first differential of output
};
