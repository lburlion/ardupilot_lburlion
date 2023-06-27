/// @file	AC_PID.cpp
/// @brief	Generic PID algorithm

#include <AP_Math/AP_Math.h>
#include "AC_PID.h"

const AP_Param::GroupInfo AC_PID::var_info[] = {
    // @Param: P
    // @DisplayName: PID Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("P", 0, AC_PID, _kp, default_kp),

    // @Param: I
    // @DisplayName: PID Integral Gain
    // @Description: I Gain which produces an output that is proportional to both the magnitude and the duration of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("I", 1, AC_PID, _ki, default_ki),

    // @Param: D
    // @DisplayName: PID Derivative Gain
    // @Description: D Gain which produces an output that is proportional to the rate of change of the error
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("D", 2, AC_PID, _kd, default_kd),

    // 3 was for uint16 IMAX

    // @Param: FF
    // @DisplayName: FF FeedForward Gain
    // @Description: FF Gain which produces an output value that is proportional to the demanded input
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FF", 4, AC_PID, _kff, default_kff),

    // @Param: IMAX
    // @DisplayName: PID Integral Maximum
    // @Description: The maximum/minimum value that the I term can output
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("IMAX", 5, AC_PID, _kimax, default_kimax),

    // 6 was for float FILT

    // 7 is for float ILMI and FF

    // index 8 was for AFF

    // @Param: FLTT
    // @DisplayName: PID Target filter frequency in Hz
    // @Description: Target filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTT", 9, AC_PID, _filt_T_hz, default_filt_T_hz),

    // @Param: FLTE
    // @DisplayName: PID Error filter frequency in Hz
    // @Description: Error filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTE", 10, AC_PID, _filt_E_hz, default_filt_E_hz),

    // @Param: FLTD
    // @DisplayName: PID Derivative term filter frequency in Hz
    // @Description: Derivative filter frequency in Hz
    // @Units: Hz
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("FLTD", 11, AC_PID, _filt_D_hz, default_filt_D_hz),

    // @Param: SMAX
    // @DisplayName: Slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced
    AP_GROUPINFO_FLAGS_DEFAULT_POINTER("SMAX", 12, AC_PID, _slew_rate_max, default_slew_rate_max),

    // MFC parameters

    // @Param: MFC_KP
    // @DisplayName: MFC PD Proportional Gain
    // @Description: P Gain which produces an output value that is proportional to the current error value
    AP_GROUPINFO("MFCP", 14, AC_PID, _mfc_kp, AC_MFC_KP_DEFAULT),

    // @Param: MFC_KD
    // @DisplayName: KD value for mfc
    // @Description: KD value for mfc
    AP_GROUPINFO("MFCD", 15, AC_PID, _mfc_kd, AC_MFC_KD_DEFAULT),

    // @Param: LAMBDA
    // @DisplayName: Lambda value for mfc
    // @Description: Lambda value for mfc
    AP_GROUPINFO("LBDA", 16, AC_PID, _lambda, AC_MFC_LAMBDA_DEFAULT),

    // @Param: MFC_GAIN
    // @DisplayName: gain value for measurement mfc
    // @Description: gain value for measurement mfc
    AP_GROUPINFO("MFCI", 17, AC_PID, _gain_i, AC_MFC_GAIN_DEFAULT),

    // @Param: MFC_M_FLIT
    // @DisplayName: measurement filter in hertz
    // @Description: measurement filter in hertz
    AP_GROUPINFO("MFCF", 18, AC_PID, _measurement_filt_hertz, AC_PID_MFILT_HZ_DEFAULT),

    // @Param: MFC_GAIN
    // @DisplayName: gain value for set point mfc
    // @Description: gain value for set point mfc
    AP_GROUPINFO("MFCS", 19, AC_PID, _gain_sp, AC_MFC_GAIN_DEFAULT),

    // @Param: MFC_GAIN
    // @DisplayName: gain value for set point mfc
    // @Description: gain value for set point mfc
    AP_GROUPINFO("MFCH", 20, AC_PID, _gain_f_hat, AC_MFC_GAIN_DEFAULT),

    // @Param: MFC_GAIN
    // @DisplayName: time step mfc
    // @Description: time step mfc
    AP_GROUPINFO("MFCT", 21, AC_PID, _mfc_dt, AC_MFC_DT_DEFAULT ),

    // @Param: MFC_GAIN
    // @DisplayName: time step mfc
    // @Description: time step mfc
    AP_GROUPINFO("MFCN", 22, AC_PID, _mfc_n, AC_MFC_DN_DEFAULT ),

    AP_GROUPEND
};

// Constructor
AC_PID::AC_PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz,
               float initial_srmax, float initial_srtau) :
    default_kp(initial_p),
    default_ki(initial_i),
    default_kd(initial_d),
    default_kff(initial_ff),
    default_kimax(initial_imax),
    default_filt_T_hz(initial_filt_T_hz),
    default_filt_E_hz(initial_filt_E_hz),
    default_filt_D_hz(initial_filt_D_hz),
    default_slew_rate_max(initial_srmax)
{
    // load parameter values from eeprom
    AP_Param::setup_object_defaults(this, var_info);

    // this param is not in the table, so its default is no loaded in the call above
    _slew_rate_tau.set(initial_srtau);

    // reset input filter to first value received
    _flags._reset_filter = true;

    memset(&_pid_info, 0, sizeof(_pid_info));

    // slew limit scaler allows for plane to use degrees/sec slew
    // limit
    _slew_limit_scale = 1;
}

// filt_T_hz - set target filter hz
void AC_PID::filt_T_hz(float hz)
{
    _filt_T_hz.set(fabsf(hz));
}

// filt_E_hz - set error filter hz
void AC_PID::filt_E_hz(float hz)
{
    _filt_E_hz.set(fabsf(hz));
}

// filt_D_hz - set derivative filter hz
void AC_PID::filt_D_hz(float hz)
{
    _filt_D_hz.set(fabsf(hz));
}

// slew_limit - set slew limit
void AC_PID::slew_limit(float smax)
{
    _slew_rate_max.set(fabsf(smax));
}

//  update_all - set target and measured inputs to PID controller and calculate outputs
//  target and error are filtered
//  the derivative is then calculated and filtered
//  the integral is then updated based on the setting of the limit flag
float AC_PID::update_all(float target, float measurement, float dt, bool limit, float boost)
{
    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _target = target;
        _error = _target - measurement;
        _derivative = 0.0f;
    } else {
        float error_last = _error;
        _target += get_filt_T_alpha(dt) * (target - _target);
        _error += get_filt_E_alpha(dt) * ((_target - measurement) - _error);

        // calculate and filter derivative
        if (is_positive(dt)) {
            float derivative = (_error - error_last) / dt;
            _derivative += get_filt_D_alpha(dt) * (derivative - _derivative);
        }
    }

    // update I term
    update_i(dt, limit);

    float P_out = (_error * _kp);
    float D_out = (_derivative * _kd);

    // calculate slew limit modifier for P+D
    _pid_info.Dmod = _slew_limiter.modifier((_pid_info.P + _pid_info.D) * _slew_limit_scale, dt);
    _pid_info.slew_rate = _slew_limiter.get_slew_rate();

    P_out *= _pid_info.Dmod;
    D_out *= _pid_info.Dmod;

    // boost output if required
    P_out *= boost;
    D_out *= boost;

    _pid_info.target = _target;
    _pid_info.actual = measurement;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;

    _last_u = P_out + _integrator + D_out;
    _pid_info.curr = _last_u;
    _sp_old_val = _target;
    _m_old_val = measurement;
    
    return P_out + _integrator + D_out;
}

// The control loop for MFC control algorithm

float AC_PID::update_all_mfc(float target, float measurement, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _target = target;
        _error = _target - measurement;
        _derivative = 0.0f;
        _sp_double_der = 0.0f;
        _f_hat = 0.0f;
        _measurement = measurement;
        _limit_checker = true;
    } else {
        float error_last = _error;
        _target += get_filt_T_alpha() * (target - _target);
        _error += get_filt_E_alpha() * ((_target - measurement) - _error);
        //_measurement += calc_lowpass_alpha_dt(_dt, _measurement_filt_hertz) * (measurement - _measurement); // filter for measurement value
        // calculate and filter derivative
        if (_dt > 0.0f) {
            float derivative = (_error - error_last) / _dt;
            _derivative += get_filt_D_alpha() * (derivative - _derivative);
        }
    }
    
    _measurement = measurement;


    
    //
    float P_out = (_error * _mfc_kp);
    //
    float D_out = (_derivative * _mfc_kd);

    // calculate slew limit modifier for P+D
    _pid_info.Dmod = _slew_limiter.modifier((_pid_info.P + _pid_info.D) * _slew_limit_scale, _dt);
    _pid_info.slew_rate = _slew_limiter.get_slew_rate();

    P_out *= _pid_info.Dmod;
    D_out *= _pid_info.Dmod;

    if(limit && _limit_checker){
        P_out = (_error * _kp);
        D_out = (_derivative * _kd);
        update_i(limit);
        _current_u = (P_out + D_out + _integrator);
    }
    else{  
        // Simpson's Approximation for Set Point Double Derivative
        _sp_double_der = F_hat_F(false, true, _target, _last_u) * _gain_sp;
        _pid_info.sp_der = _sp_double_der/ _lambda;
        update_i(limit);
        // update f hat value
        _f_hat = F_hat_F(false, false, _measurement, _last_u) * _gain_f_hat;
        _pid_info.m_der = F_hat_F(false, true, _measurement, _last_u) * _gain_f_hat;
        _current_u = ((_sp_double_der + P_out + _integrator * _gain_i  +  D_out - _f_hat) / _lambda);
        _limit_checker = false;
    }

    _last_u = _current_u;  


    
    _pid_info.target = _target;
    _pid_info.actual = _measurement;
    _pid_info.error = _error;
    _pid_info.P = P_out/ _lambda;
    _pid_info.D = D_out/ _lambda;
    _pid_info.I = _integrator * _gain_i / _lambda;
    _pid_info.f_hat = -_f_hat/ _lambda;
    _pid_info.curr = _current_u;
    _pid_info.limit = limit;
   

    return _current_u;
}

float AC_PID::update_all_mfc_yaw(float target, float measurement, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(target) || !isfinite(measurement)) {
        return 0.0f;
    }

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _target = target;
        _error = _target - measurement;
        _derivative = 0.0f;
        _sp_double_der = 0.0f;
        _f_hat = 0.0f;
        _measurement = measurement;
        _limit_checker = true;
    } else {
        float error_last = _error;
        _target += get_filt_T_alpha() * (target - _target);
        _error += get_filt_E_alpha() * ((_target - measurement) - _error);
        //_measurement += calc_lowpass_alpha_dt(_dt, _measurement_filt_hertz) * (measurement - _measurement); // filter for measurement value
        // calculate and filter derivative
        _measurement = measurement;
        if (_dt > 0.0f) {
            float derivative = (_error - error_last) / _dt;
            _derivative += get_filt_D_alpha() * (derivative - _derivative);
        }
    }

    //
    float P_out = (_error * _mfc_kp);
    //
    float D_out = (_derivative * _mfc_kd);

    // calculate slew limit modifier for P+D
    _pid_info.Dmod = _slew_limiter.modifier((_pid_info.P + _pid_info.D) * _slew_limit_scale, _dt);
    _pid_info.slew_rate = _slew_limiter.get_slew_rate();

    P_out *= _pid_info.Dmod;
    D_out *= _pid_info.Dmod;

    if(limit && _limit_checker){
        P_out = (_error * _kp);
        D_out = (_derivative * _kd);
        update_i(limit);
        _current_u = (P_out + D_out + _integrator);
    }
    else{
        // update f hat value
        update_i(limit);
        // Simpson's Approximation for Set Point Double Derivative
        _sp_double_der = F_hat_F(true, true, _target, _last_u) * _gain_sp;
        _pid_info.sp_der = _sp_double_der;
        _f_hat = F_hat_F(true, false, _measurement, _last_u) * _gain_f_hat ;
        _pid_info.m_der = F_hat_F(true, true, _measurement, _last_u) * _gain_f_hat;
        _current_u = ((_sp_double_der + P_out + _integrator * _gain_i + D_out - _f_hat) / _lambda);
        _limit_checker = false;
    }
    _last_u = _current_u;  // update the last u value for next loop;


    
    _pid_info.target = _target;
    _pid_info.actual = _measurement;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;
    _pid_info.I =  _integrator * _gain_i;
    _pid_info.f_hat = _f_hat;
    _pid_info.curr = _current_u;
    _pid_info.limit = limit;
   

    return _current_u;
}


//  update_error - set error input to PID controller and calculate outputs
//  target is set to zero and error is set and filtered
//  the derivative then is calculated and filtered
//  the integral is then updated based on the setting of the limit flag
//  Target and Measured must be set manually for logging purposes.
// todo: remove function when it is no longer used.

float AC_PID::double_derivative_set_point(float set_point)
{

    float rtrn_val = _gain_sp * (set_point - _sp_old_val) / _dt; // first derivative
    //rtrn_val += get_filt_D_alpha() * (set_point - _sp_old_val); // filter the first derivative value
    _sp_old_val = set_point; // store the old sp value for next loop
    float temp_sp_first_der_old_val = rtrn_val; // store the the first derivative in a tempa variable
    rtrn_val = _gain_sp * (rtrn_val - _sp_first_der_old_val) / _dt;  // second derivative
    //rtrn_val += get_filt_D_alpha() * (rtrn_val - _sp_first_der_old_val); // filter the second derivative value
    _sp_first_der_old_val = temp_sp_first_der_old_val; // store the old first derivative value for next loop

    return rtrn_val;
}

float AC_PID::double_derivative_measurement(float measurement)
{
    float rtrn_val = _gain_i * (measurement - _m_old_val) / _dt; 
    //rtrn_val += get_filt_D_alpha() * (measurement - _m_old_val);
    _m_old_val = measurement;
    float temp_m_first_der_old_val = rtrn_val;
    rtrn_val = _gain_i * (rtrn_val - _m_first_der_old_val) / _dt;
    //rtrn_val += get_filt_D_alpha() * (rtrn_val - _m_first_der_old_val);
    _m_first_der_old_val = temp_m_first_der_old_val;


    return rtrn_val;
}

float AC_PID::m_der_average(float new_num)
{
    int i = 0;
    for (i = 0; i < 4; i++){
        _m_der_set[i+1] = _m_der_set[i];
    }
    _m_der_set[0] = new_num;

    float sum = 0;

    for(i = 0; i < 5; i++) {
        sum += _m_der_set[i];
    }

    return sum/5;
}

float AC_PID::F_hat_F(bool y, bool z, float _f_measurement, float _last_f_u){

    int n = _mfc_n; 
    
    float x[n];
    
    x[n - 1] = _mfc_dt;
    
    float dx = (_mfc_dt) / n;
    
    float Tot_e[n/2 + 1];
    
    float Tot_o[n/2 + 1];
    
    for (int j = 0;j < n - 1 ; j++)
    {    
        x[j] = (dx * j);
    }
    
    for (int i = 0; i < n/2 + 1; i++) 
    {    
        Tot_e[i+1] = 4*F_hat(x[2*i + 1], y, z, _f_measurement, _last_f_u);
        Tot_o[i] = 2*F_hat(x[2*i], y, z, _f_measurement, _last_f_u);
    }
    
    Tot_e[0] = 0;
    Tot_e[n/2] = 4*F_hat(x[1], y, z, _f_measurement, _last_f_u);
    Tot_o[0] = F_hat(x[0], y, z, _f_measurement, _last_f_u);
    Tot_o[n/2] = F_hat(x[n - 1], y, z, _f_measurement, _last_f_u);
    float sum = 0;
    
    for(int k = 0; k < n/2 + 1; k++) {
        sum += (Tot_e[k] + Tot_o[k])*(dx/3);
    }

    return sum;

}

float AC_PID::F_hat(float x, bool y, bool z, float _f1_measurement, float _last_f1_u){

    if (y && !z){

         _F_hat_calc = (6/(powf(_mfc_dt,3.0)))*((_mfc_dt - 2*x)*_f1_measurement -  (_lambda * (_mfc_dt - x) * x * _last_f1_u));

         return _F_hat_calc;

    }else if(!y && !z) {

         _F_hat_calc = (60/(powf(_mfc_dt,5.0)))*(((powf(_mfc_dt,2.0) - (6*(_mfc_dt - x))*_mfc_dt + (6* powf((_mfc_dt - x), 2)))*_f1_measurement) -  ((_lambda/2) * powf((_mfc_dt - x), 2) * powf((_mfc_dt - (_mfc_dt - x)), 2) * _last_f1_u));

         return _F_hat_calc;
         
    }
    else if(y && z){
        
        _F_hat_calc = (6/(powf(_mfc_dt,3.0)))*((_mfc_dt - 2*x) * _f1_measurement);

        return _F_hat_calc;
        
    }
    else {

        _F_hat_calc = (60/(powf(_mfc_dt,5.0)))*(((powf(_mfc_dt,2.0) - (6*(_mfc_dt - x))*_mfc_dt + (6* powf((_mfc_dt - x), 2))) * _f1_measurement));

        return _F_hat_calc;
    }
}

//  update_error - set error input to PID controller and calculate outputs
//  target is set to zero and error is set and filtered
//  the derivative then is calculated and filtered
//  the integral is then updated based on the setting of the limit flag
//  Target and Measured must be set manually for logging purposes.
// todo: remove function when it is no longer used.
float AC_PID::update_error(float error, float dt, bool limit)
{
    // don't process inf or NaN
    if (!isfinite(error)) {
        return 0.0f;
    }

    _target = 0.0f;

    // reset input filter to value received
    if (_flags._reset_filter) {
        _flags._reset_filter = false;
        _error = error;
        _derivative = 0.0f;
    } else {
        float error_last = _error;
        _error += get_filt_E_alpha(dt) * (error - _error);

        // calculate and filter derivative
        if (is_positive(dt)) {
            float derivative = (_error - error_last) / dt;
            _derivative += get_filt_D_alpha(dt) * (derivative - _derivative);
        }
    }

    // update I term
    update_i(dt, limit);

    float P_out = (_error * _kp);
    float D_out = (_derivative * _kd);

    // calculate slew limit modifier for P+D
    _pid_info.Dmod = _slew_limiter.modifier((_pid_info.P + _pid_info.D) * _slew_limit_scale, dt);
    _pid_info.slew_rate = _slew_limiter.get_slew_rate();

    P_out *= _pid_info.Dmod;
    D_out *= _pid_info.Dmod;
    
    _pid_info.target = 0.0f;
    _pid_info.actual = 0.0f;
    _pid_info.error = _error;
    _pid_info.P = P_out;
    _pid_info.D = D_out;

    return P_out + _integrator + D_out;
}

//  update_i - update the integral
//  If the limit flag is set the integral is only allowed to shrink
void AC_PID::update_i(float dt, bool limit)
{
    if (!is_zero(_ki) && is_positive(dt)) {
        // Ensure that integrator can only be reduced if the output is saturated
        if (!limit || ((is_positive(_integrator) && is_negative(_error)) || (is_negative(_integrator) && is_positive(_error)))) {
            _integrator += ((float)_error * _ki) * dt;
            _integrator = constrain_float(_integrator, -_kimax, _kimax);
        }
    } else {
        _integrator = 0.0f;
    }
    _pid_info.I = _integrator;
    _pid_info.limit = limit;
}

float AC_PID::get_p() const
{
    return _error * _kp;
}

float AC_PID::get_i() const
{
    return _integrator;
}

float AC_PID::get_d() const
{
    return _kd * _derivative;
}

float AC_PID::get_ff()
{
    _pid_info.FF = _target * _kff;
    return _target * _kff;
}

void AC_PID::reset_I()
{
    _integrator = 0.0;
}

void AC_PID::load_gains()
{
    _kp.load();
    _ki.load();
    _kd.load();
    _kff.load();
    _kimax.load();
    _kimax.set(fabsf(_kimax));
    _filt_T_hz.load();
    _filt_E_hz.load();
    _filt_D_hz.load();
}

// save_gains - save gains to eeprom
void AC_PID::save_gains()
{
    _kp.save();
    _ki.save();
    _kd.save();
    _kff.save();
    _kimax.save();
    _filt_T_hz.save();
    _filt_E_hz.save();
    _filt_D_hz.save();
}

/// Overload the function call operator to permit easy initialisation
void AC_PID::operator()(float p_val, float i_val, float d_val, float ff_val, float imax_val, float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz)
{
    _kp.set(p_val);
    _ki.set(i_val);
    _kd.set(d_val);
    _kff.set(ff_val);
    _kimax.set(fabsf(imax_val));
    _filt_T_hz.set(input_filt_T_hz);
    _filt_E_hz.set(input_filt_E_hz);
    _filt_D_hz.set(input_filt_D_hz);
}

// get_filt_T_alpha - get the target filter alpha
float AC_PID::get_filt_T_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_T_hz);
}

// get_filt_E_alpha - get the error filter alpha
float AC_PID::get_filt_E_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_E_hz);
}

// get_filt_D_alpha - get the derivative filter alpha
float AC_PID::get_filt_D_alpha(float dt) const
{
    return calc_lowpass_alpha_dt(dt, _filt_D_hz);
}

void AC_PID::set_integrator(float target, float measurement, float integrator)
{
    set_integrator(target - measurement, integrator);
}

void AC_PID::set_integrator(float error, float integrator)
{
    _integrator = constrain_float(integrator - error * _kp, -_kimax, _kimax);
}

void AC_PID::set_integrator(float integrator)
{
    _integrator = constrain_float(integrator, -_kimax, _kimax);
}

void AC_PID::relax_integrator(float integrator, float dt, float time_constant)
{
    integrator = constrain_float(integrator, -_kimax, _kimax);
    if (is_positive(dt)) {
        _integrator = _integrator + (integrator - _integrator) * (dt / (dt + time_constant));
    }
}
