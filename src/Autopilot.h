/*
 * Output of all controllers are assumed to be in centi-degrees.
 * This output is automatically converted into us pulses for servo control.
 */

#pragma once
#include "Arduino.h"




const float GRAVITY = 9.80665; // m/s2

const int MAX_F = 20;
const int MIN_F = 1;

const float MAX_P = 10000;
const float MIN_P = 0.1;

const float MAX_I = 100000;
const float MIN_I = 0.0;

const float MAX_D = 10000;
const float MIN_D = 0.0;

const int MAX_RATE_UP = 100;
const int MIN_RATE_UP = 0;

const int MAX_RATE_DN = 100;
const int MIN_RATE_DN = 0;

const int MAX_I_LIMIT = 4500;
const int MIN_I_LIMIT = 0;

const int MAX_SERVO = 2000;
const int MIN_SERVO = 1000;




float float_constrain(float input, float min, float max);
float float_map(float x, float in_min, float in_max, float out_min, float out_max);
int centiDeg_to_us(int controller_output_, int servoMin_, int servoMax_);




struct control_params {
	float setpoint;   // Degrees
	int maxRate_up;   // Degrees per Sec
	int maxRate_down; // Degrees per Sec
	float kp;         // Unitless
	float ki;         // Unitless
	float kd;         // Unitless
	float sampleRate; // Hz
	int servoMax;     // ms pulswidth
	int servoMin;     // ms pulswidth
};




class pitch_controller
{
public:
	bool status = false;




	void begin(control_params params);
	void update(control_params params);
	void reset();
	float compute(float pitchAngle, // Degrees
	              float rollAngle,  // Degrees
	              float airspeed);  // m/s

	float get_P_Component();
	float get_I_Component();
	float get_D_Component();




private:
	const float MAX_SETPOINT = 90.0;
	const float MIN_SETPOINT = -90.0;




	unsigned long sampleTimer_previous = 0;
	unsigned long sampleTimer_current  = 0;
	unsigned long sample_time_actual   = 0;

	unsigned int samplePeriod_ms;
	float samplePeriod_s;

	float setpoint;
	float kp;
	float ki;
	float kd;
	float roll_comp;

	int servoMax;
	int servoMin;

	float p_val;
	float i_val;
	float d_val;

	int maxRate_up;
	int maxRate_down;
	int i_limit         = 1000; // Centi-Degrees
	float error         = 0;
	float previousError = 0;
	float summedError   = 0;



	
	float roll_compensation(float rollAngle, float airspeed);
	float find_as_scaler(float airspeed);
	float p_component();
	float i_component();
	float d_component();
};




class roll_controller
{
public:
	bool status = false;




	void begin(control_params params);
	void update(control_params params);
	void reset();
	float compute(float rollAngle,  // Degrees
	              float airspeed);  // m/s

	float get_P_Component();
	float get_I_Component();
	float get_D_Component();




private:
	const float MAX_SETPOINT = 180;
	const float MIN_SETPOINT = -180;




	unsigned long sampleTimer_previous = 0;
	unsigned long sampleTimer_current  = 0;
	unsigned long sample_time_actual   = 0;

	unsigned int samplePeriod_ms;
	float samplePeriod_s;

	float setpoint;
	float kp;
	float ki;
	float kd;

	int servoMax;
	int servoMin;

	float p_val;
	float i_val;
	float d_val;

	int i_limit = 1500; // Centi-Degrees
	float error = 0;
	float previousError = 0;
	float summedError = 0;
	float biasedError = 0;




	float find_kp(float kp_,
	              float ki_,
	              float kd_,
	              float samplePeriod_s);
	float find_ki(float ki_, float samplePeriod_s);
	float p_component();
	float i_component();
	float d_component();
};