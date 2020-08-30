#include "Autopilot.h"




float float_constrain(float input, float min, float max)
{
	if (input > max)
		return max;
	else if (input < min)
		return min;
	else
		return input;
}




float float_map(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}




int centiDeg_to_us(int controller_output_, int outputMin_, int outputMax_)
{
	// Map high is 18000 (180 * 100) since controller_output is in centi-degrees
	return (int)float_constrain(float_map(controller_output_, 0, 18000, outputMin_, outputMax_), outputMin_, outputMax_);
}




void basic_controller::begin(control_params params)
{
	update(params);
	reset();
}




void basic_controller::update(control_params params)
{
	setpoint = params.setpoint;
	samplePeriod_s = 1 / float_constrain(params.sampleRate, MIN_F, MAX_F);

	samplePeriod_ms = 1000 * samplePeriod_s;

	kp = float_constrain(params.kp, MIN_P, MAX_P);
	ki = float_constrain(params.ki, MIN_I, MAX_I);
	kd = float_constrain(params.kd, MIN_D, MAX_D);

	outputMax = params.outputMax;
	outputMin = params.outputMin;
}




void basic_controller::reset()
{
	error = 0;
	previousError = 0;
	summedError = 0;

	timer.begin(samplePeriod_ms);
}




float basic_controller::get_P_Component()
{
	return p_val;
}




float basic_controller::get_I_Component()
{
	return i_val;
}




float basic_controller::get_D_Component()
{
	return d_val;
}




float basic_controller::p_component()
{
	return kp * error;
}




float basic_controller::i_component()
{
	summedError += ((error + previousError) / 2.0) * (timer.timeDiff / 1000.0);
	summedError = float_constrain(summedError, -i_limit / ki, i_limit / ki);

	return ki * summedError;
}




float basic_controller::d_component()
{
	return kd * (error - previousError) / timer.timeDiff;
}
