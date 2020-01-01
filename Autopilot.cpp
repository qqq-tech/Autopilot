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




int centiDeg_to_us(int controller_output_, int servoMin_, int servoMax_)
{
	// Map high is 18000 (180 * 100) since controller_output is in centi-degrees
	return (int)float_constrain(float_map(controller_output_, 0, 18000, servoMin_, servoMax_), servoMin_, servoMax_);
}




void pitch_controller::begin(control_params params)
{
	update(params);
	reset();
}




void pitch_controller::update(control_params params)
{
	setpoint        = float_constrain(params.setpoint, MIN_SETPOINT, MAX_SETPOINT);
	maxRate_up      = float_constrain(abs(params.maxRate_up), MIN_RATE_UP, MAX_RATE_UP);
	maxRate_down    = float_constrain(abs(params.maxRate_down), MIN_RATE_DN, MAX_RATE_DN);
	samplePeriod_s  = 1 / float_constrain(params.sampleRate, MIN_F, MAX_F);
	samplePeriod_ms = 1000 * samplePeriod_s;
	kp              = float_constrain(find_kp(params.kp, params.ki, params.kd, samplePeriod_s), MIN_P, MAX_P);
	ki              = float_constrain(find_ki(params.ki, samplePeriod_s), MIN_I, MAX_I);
	kd              = float_constrain(params.kd, MIN_D, MAX_D);
	servoMax        = params.servoMax;
	servoMin        = params.servoMin;
}




void pitch_controller::reset()
{
	sampleTimer_previous = 0;
	sampleTimer_current  = 0;
	sample_time_actual   = 0;
	error = 0;
	previousError  = 0;
	summedError    = 0;
	sampleTimer_current  = millis();
	sampleTimer_previous = sampleTimer_current;
}




float pitch_controller::compute(float pitchAngle, // Degrees
                                float rollAngle,  // Degrees
                                float airspeed)   // m/s
{
	float roll_bias;
	float controller_output;

	sampleTimer_current = millis();
	sample_time_actual  = sampleTimer_current - sampleTimer_previous;
	
	if (sample_time_actual >= samplePeriod_ms)
	{
		sampleTimer_previous += samplePeriod_ms;
		
		previousError = error;
		error = setpoint - pitchAngle;

		p_val = constrain(p_component(), -(10 * (long)samplePeriod_ms), 10 * (long)samplePeriod_ms);
		i_val = i_component();
		d_val = d_component();
		
		controller_output = constrain((p_val + i_val + d_val) + (18000 / 2), 0, 18000);

		Serial1.println(summedError, 10);
		Serial1.print("setpoint: "); Serial1.println(setpoint);
		Serial1.print("previousError: "); Serial1.println(previousError);
		Serial1.print("error: "); Serial1.println(error);
		Serial1.print("p_val: "); Serial1.println(p_val);
		Serial1.print("i_val: "); Serial1.println(i_val);
		Serial1.print("d_val: "); Serial1.println(d_val);
		Serial1.println();

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




float pitch_controller::find_kp(float kp_,
                                float ki_,
                                float kd_,
                                float samplePeriod)
{
	return kp_;
}




float pitch_controller::find_ki(float ki_, float samplePeriod)
{
	return ki_;
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
	summedError  = float_constrain(summedError, -i_limit / ki, i_limit / ki);

	return ki * summedError;
}




float pitch_controller::d_component()
{
	return kd * (error - previousError) / sample_time_actual;
}




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

		/*Serial1.print("previousError: "); Serial1.println(previousError);
		Serial1.print("error: "); Serial1.println(error);
		Serial1.print("p_val: "); Serial1.println(p_val);
		Serial1.print("i_val: "); Serial1.println(i_val);
		Serial1.print("d_val: "); Serial1.println(d_val);
		Serial1.print("controller_output: "); Serial1.println(controller_output);
		Serial1.print("mapped_output: "); Serial1.println(centiDeg_to_us(controller_output, servoMin, servoMax));
		Serial1.println();*/

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