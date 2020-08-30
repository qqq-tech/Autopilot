#pragma once
#include "Arduino.h"
#include "FireTimer.h"




const int MAX_F = 20;
const int MIN_F = 1;

const float MAX_P = 10000;
const float MIN_P = 0.1;

const float MAX_I = 100000;
const float MIN_I = 0.0;

const float MAX_D = 10000;
const float MIN_D = 0.0;

const int MAX_I_LIMIT = 4500;
const int MIN_I_LIMIT = 0;




float float_constrain(float input, float min, float max);
float float_map(float x, float in_min, float in_max, float out_min, float out_max);




struct control_params {
	float setpoint;   // Degrees
	float kp;         // Unitless
	float ki;         // Unitless
	float kd;         // Unitless
	float sampleRate; // Hz
	float outputMax;  // Unitless
	float outputMin;  // Unitless
};

struct state_params {
	float pitch;
	float roll;
	float hdg;
	float alt;
	float ias;
	float lat;
	float lon;
	float flaps;
	float gear;
};




class basic_controller
{
public:
	bool status = false;

	float setpoint;

	float kp;
	float ki;
	float kd;




	void begin(control_params params);
	void update(control_params params);
	void reset();
	virtual float compute(state_params state) = 0;
	
	float get_P_Component();
	float get_I_Component();
	float get_D_Component();




protected:
	unsigned int samplePeriod_ms;
	float samplePeriod_s;

	float p_val;
	float i_val;
	float d_val;

	float outputMax;
	float outputMin;

	int i_limit = 55; // ms - approximately 10 degrees of output throw

	float error = 0;
	float previousError = 0;
	float summedError = 0;

	FireTimer timer;




	float p_component();
	float i_component();
	float d_component();
};




class pitch_controller : public basic_controller
{
public:
	float compute(state_params state);




private:
	float roll_comp;



	
	float roll_compensation(float controllerOutput, state_params state);
};




class roll_controller : public basic_controller
{
public:
	float compute(state_params state);
};




class heading_controller : public basic_controller
{
public:
	float compute(state_params state);
};




class altitude_controller : public basic_controller
{
public:
	float compute(state_params state);
};

