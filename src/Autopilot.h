#pragma once
#include "Arduino.h"
#include "FireTimer.h"




const float EARTH_RADIUS_KM = 6378.137;

const int MAX_F = 20;
const int MIN_F = 1;

const float MAX_P = 100;
const float MIN_P = 0.1;

const float MAX_I = 1000;
const float MIN_I = 0.0;

const float MAX_D = 100;
const float MIN_D = 0.0;

const int MAX_I_LIMIT = 45;
const int MIN_I_LIMIT = 0.0;




float float_constrain(const float& input, const float& min, const float& max);
float float_map(const float& x, const float& in_min, const float& in_max, const float& out_min, const float& out_max);
float find_heading(const float& lat_1, const float& lon_1, const float& lat_2, const float& lon_2);
float find_distance(const float& lat_1, const float& lon_1, const float& lat_2, const float& lon_2);
void find_coord(const float& lat, const float& lon, float& lat_2, float& lon_2, const float& distance, const float& bearing);




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
	float roll;
	float pitch;
	float hdg;
	float alt;
	float lat;
	float lon;
	float UTC_year;
	float UTC_month;
	float UTC_day;
	float UTC_hour;
	float UTC_minute;
	float UTC_second;
	float sog;
	float cog;
	float ias;
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




	void begin(const control_params& params);
	void update(const control_params& params);
	void reset();
	virtual float compute(const state_params& state) = 0;
	
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
	float compute(const state_params& state);




private:
	float roll_comp;



	
	float roll_compensation(const float& controllerOutput, const state_params& state);
};




class roll_controller : public basic_controller
{
public:
	float compute(const state_params& state);
};




class heading_controller : public basic_controller
{
public:
	float compute(const state_params& state);




private:
	float inputScrub(const state_params& state);
};




class altitude_controller : public basic_controller
{
public:
	float compute(const state_params& state);
};




class ias_controller : public basic_controller
{
public:
	float compute(const state_params& state);
};

