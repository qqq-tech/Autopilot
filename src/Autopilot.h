#pragma once
#include "Arduino.h"
#include "FireTimer.h"




const float EARTH_RADIUS_M  = 6378100.0;
const float EARTH_RADIUS_KM = EARTH_RADIUS_M / 1000;
const float EARTH_GRAVITY   = 9.80665;

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




float slope(const float& xi, const float& yi, const float& xf, const float& yf);
float float_constrain(const float& input, const float& min, const float& max);
float float_map(const float& x, const float& in_min, const float& in_max, const float& out_min, const float& out_max);
float heading(const float& lat_1, const float& lon_1, const float& lat_2, const float& lon_2);
float distance(const float& lat_1, const float& lon_1, const float& lat_2, const float& lon_2, const bool& km = false);
void coord(const float& lat, const float& lon, float& lat_2, float& lon_2, const float& distance, const float& bearing, const bool& km = false);




struct control_params {
	float setpoint;   // Degrees
	float kp;         // Unitless
	float ki;         // Unitless
	float kd;         // Unitless
	float sampleRate; // Hz
	float outputMax;  // Unitless
	float outputMin;  // Unitless
};

struct pilsim_state_params {
	float roll;
	float pitch;
	float hdg;
	float alt;
	float lat;
	float lon;
	float ias;
	float flaps;
	float gear;
};

struct state_params {
	float roll;
	float pitch;

	float hdg;
	float hdg_comp;
	float hdg_imu;

	float cog;
	float cog_gps;
	float cog_gps_calc;

	float alt;
	float alt_gps;
	float alt_baro;
	float alt_lidar;

	float lat;
	float lon;
	float prev_lat;
	float prev_lon;

	uint16_t UTC_year;
	uint8_t UTC_month;
	uint8_t UTC_day;
	uint8_t UTC_hour;
	uint8_t UTC_minute;
	uint8_t UTC_second;

	float ias;
	float ias_pitot;
	float ias_gps;
	float ias_gps_calc;

	bool flaps;
	bool gear;
};

struct point {
	float maxRoll;    // °
	float minTurnRad; // m
	float hdgAngRate; // °/s
	float hitRadius;  // m

	float speed;   // Speed at point m/s
	float heading; // Heading at point °
	float lat;     // Point lat °
	float lon;     // Point lon °

	float rc_lat;  // Right Turn Circle lat °
	float rc_lon;  // Right Turn Circle lon °
	float lc_lat;  // Left Turn Circle lat °
	float lc_lon;  // Left Turn Circle lon °
	float c_lat;   // Selected Turn Circle lat °
	float c_lon;   // Selected Turn Circle lon °
	float e_lat;   // Enter/exit lat °
	float e_lon;   // Enter/exit lon °
};

enum dubins {
	LSR, // Left Straight Right
	RSL, // Right Straight Left
	RSR, // Right Straight Right
	LSL  // Left Straight Left
};

struct nav_frame {
	dubins path; // Dubins path type

	float sps; // Straight Path Speed m/s

	point ni; // Current point
	point nf; // Next point
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
	virtual float compute(const pilsim_state_params& state) = 0;
	
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
	float summedError   = 0;

	FireTimer timer;




	float p_component();
	float i_component();
	float d_component();
};




class pitch_controller : public basic_controller
{
public:
	float compute(const state_params& state);
	float compute(const pilsim_state_params& state);




private:
	float roll_comp;



	
	float roll_compensation(const float& controllerOutput, const state_params& state);
	float roll_compensation(const float& controllerOutput, const pilsim_state_params& state);
};




class roll_controller : public basic_controller
{
public:
	float compute(const state_params& state);
	float compute(const pilsim_state_params& state);
};




class heading_controller : public basic_controller
{
public:
	float compute(const state_params& state);
	float compute(const pilsim_state_params& state);




private:
	float inputScrub(const state_params& state);
	float inputScrub(const pilsim_state_params& state);
};




class altitude_controller : public basic_controller
{
public:
	float compute(const state_params& state);
	float compute(const pilsim_state_params& state);
};




class ias_controller : public basic_controller
{
public:
	float compute(const state_params& state);
	float compute(const pilsim_state_params& state);
};




class combined_controller
{
public:
	pitch_controller    pitchController;
	roll_controller     rollController;
	heading_controller  headingController;
	altitude_controller altitudeController;
	ias_controller      iasController;

	state_params state;
	nav_frame    navFrame;




	void processFrame();
	void processFrame(nav_frame& frame);
	void findHAR(point& curPoint); // Heading Angular Rate
	void findMTR(point& curPoint); // Minimum Turn Radius
	void findTurnCenters(point& curPoint);
	void findPath(nav_frame& frame); 
	void findEPts(nav_frame& frame); // Enter/Exit Point locations
};
