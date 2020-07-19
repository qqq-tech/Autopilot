#include "Autopilot.h"




void pitch_controller::begin(control_params params)
{
	update(params);
	reset();
}




void pitch_controller::update(control_params params)
{
	setpoint        = float_constrain(params.setpoint,          MIN_SETPOINT, MAX_SETPOINT);
	maxRate_up      = float_constrain(abs(params.maxRate_up),   MIN_RATE_UP,  MAX_RATE_UP);
	maxRate_down    = float_constrain(abs(params.maxRate_down), MIN_RATE_DN,  MAX_RATE_DN);
	samplePeriod_s  = 1 / float_constrain(params.sampleRate,    MIN_F,        MAX_F);

	samplePeriod_ms = 1000 * samplePeriod_s;

	kp = float_constrain(params.kp, MIN_P, MAX_P);
	ki = float_constrain(params.ki, MIN_I, MAX_I);
	kd = float_constrain(params.kd, MIN_D, MAX_D);

	servoMax = params.servoMax;
	servoMin = params.servoMin;
}




void pitch_controller::reset()
{
	sampleTimer_previous = 0;
	sampleTimer_current = 0;
	sample_time_actual = 0;

	error = 0;
	previousError = 0;
	summedError = 0;

	sampleTimer_current = millis();
	sampleTimer_previous = sampleTimer_current;
}




float pitch_controller::compute(float pitchAngle, // Degrees
                                float rollAngle,  // Degrees
                                float airspeed)   // m/s
{
	float roll_bias;
	float controller_output;

	sampleTimer_current = millis();
	sample_time_actual = sampleTimer_current - sampleTimer_previous;

	if (sample_time_actual >= samplePeriod_ms)
	{
		sampleTimer_previous += samplePeriod_ms;

		previousError = error;
		error = setpoint - pitchAngle;

		p_val = constrain(p_component(), -(10 * (long)samplePeriod_ms), 10 * (long)samplePeriod_ms);
		i_val = i_component();
		d_val = d_component();

		controller_output = constrain((p_val + i_val + d_val) + (18000 / 2), 0, 18000);

		status = true;
		return centiDeg_to_us(controller_output, servoMin, servoMax);
	}

	status = false;
	return 0;
}




float pitch_controller::get_P_Component()
{
	return p_val;
}




float pitch_controller::get_I_Component()
{
	return i_val;
}




float pitch_controller::get_D_Component()
{
	return d_val;
}




float pitch_controller::roll_compensation(float rollAngle, float airspeed)
{
	return 0;
}




float pitch_controller::p_component()
{
	return kp * error;
}




float pitch_controller::i_component()
{
	summedError += ((error + previousError) / 2.0) * (sample_time_actual / 1000.0);
	summedError = float_constrain(summedError, -i_limit / ki, i_limit / ki);

	return ki * summedError;
}




float pitch_controller::d_component()
{
	return kd * (error - previousError) / sample_time_actual;
}