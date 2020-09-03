#include "Autopilot.h"




float float_constrain(const float& input, const float& min, const float& max)
{
	if (input > max)
		return max;
	else if (input < min)
		return min;
	else
		return input;
}




float float_map(const float& x, const float& in_min, const float& in_max, const float& out_min, const float& out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}




float find_heading(const float& lat_1, const float& lon_1, const float& lat_2, const float& lon_2)
{
	float deltaLon_r = radians(lon_2 - lon_1);
	float lat_1_r = radians(lat_1);
	float lat_2_r = radians(lat_2);

	float x = cos(lat_2_r) * sin(deltaLon_r);
	float y = cos(lat_1_r) * sin(lat_2_r) - sin(lat_1_r) * cos(lat_2_r) * cos(deltaLon_r);

	return fmod((degrees(atan2(x, y)) + 360), 360);
}




float find_distance(const float& lat_1, const float& lon_1, const float& lat_2, const float& lon_2)
{
	float lat_1_r = radians(lat_1);
	float lon_1_r = radians(lon_1);
	float lat_2_r = radians(lat_2);
	float lon_2_r = radians(lon_2);

	float deltaLat_r = lat_2_r - lat_1_r;
	float deltaLon_r = lon_2_r - lon_1_r;

	float a = (sin(deltaLat_r / 2) * sin(deltaLat_r / 2)) + cos(lat_1_r) * cos(lat_2_r) * (sin(deltaLon_r / 2)) * (sin(deltaLon_r / 2));

	return 2 * EARTH_RADIUS_KM * atan2(sqrt(a), sqrt(1 - a));
}




void find_coord(const float& lat_1, const float& lon_1, float& lat_2, float& lon_2, const float& distance, const float& bearing)
{
	float bearing_r = radians(bearing);
	float adj_dist = distance / EARTH_RADIUS_KM;

	float lat_1_r = radians(lat_1);
	float lon_1_r = radians(lon_1);

	lat_2 = asin(sin(lat_1_r) * cos(adj_dist) + cos(lat_1_r) * sin(adj_dist) * cos(bearing_r));
	lon_2 = lon_1_r + atan2(sin(bearing_r) * sin(adj_dist) * cos(lat_1_r), cos(adj_dist) - sin(lat_1_r) * sin(lat_2));

	lat_2 = degrees(lat_2);
	lon_2 = degrees(lon_2);
}




void basic_controller::begin(const control_params& params)
{
	update(params);
	reset();
}




void basic_controller::update(const control_params& params)
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
