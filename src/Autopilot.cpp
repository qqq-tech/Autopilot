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
