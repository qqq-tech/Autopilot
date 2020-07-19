#include "Autopilot.h"




void roll_controller::begin(control_params params)
{
	update(params);
	reset();
}




void roll_controller::update(control_params params)
{
	setpoint = float_constrain(params.setpoint, MIN_SETPOINT, MAX_SETPOINT);
	samplePeriod_s = 1 / float_constrain(params.sampleRate, MIN_F, MAX_F);
	samplePeriod_ms = 1000 * samplePeriod_s;
	kp = float_constrain(find_kp(params.kp, params.ki, params.kd, samplePeriod_s), MIN_P, MAX_P);
	ki = float_constrain(find_ki(params.ki, samplePeriod_s), MIN_I, MAX_I);
	kd = float_constrain(params.kd, MIN_D, MAX_D);
	servoMax = params.servoMax;
	servoMin = params.servoMin;
}




void roll_controller::reset()
{
	sampleTimer_previous = 0;
	sampleTimer_current = 0;
	sample_time_actual = 0;
	error = 0;
	previousError = 0;
	summedError = 0;
	biasedError = 0;
	sampleTimer_current = millis();
	sampleTimer_previous = sampleTimer_current;
}




float roll_controller::compute(float rollAngle, // Degrees
	float airspeed)  // m/s
{
	float controller_output;

	sampleTimer_current = millis();
	sample_time_actual = sampleTimer_current - sampleTimer_previous;

	if (sample_time_actual >= samplePeriod_ms)
	{
		sampleTimer_previous += samplePeriod_ms;

		previousError = error;
		error = setpoint - rollAngle;

		p_val = p_component();
		i_val = i_component();
		d_val = d_component();

		controller_output = constrain((p_val + i_val + d_val) + (18000 / 2), 0, 18000);

		status = true;
		return centiDeg_to_us(controller_output, servoMin, servoMax);
	}

	status = false;
	return 0;
}




float roll_controller::get_P_Component()
{
	return p_val;
}




float roll_controller::get_I_Component()
{
	return i_val;
}




float roll_controller::get_D_Component()
{
	return d_val;
}




float roll_controller::find_kp(float kp_,
	float ki_,
	float kd_,
	float samplePeriod)
{
	return (kp_ - (ki_ * samplePeriod)) * (samplePeriod - kd_);
}




float roll_controller::find_ki(float ki_, float samplePeriod)
{
	return ki_ * samplePeriod;
}




float roll_controller::p_component()
{
	return kp * error;
}




float roll_controller::i_component()
{
	summedError += ((error - previousError) / 2) * (sample_time_actual / 1000);
	summedError = float_constrain(summedError, -i_limit, i_limit);

	return ki * summedError;
}




float roll_controller::d_component()
{
	return kd * (error - previousError) / sample_time_actual;
}
