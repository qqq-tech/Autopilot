#include "Autopilot.h"




float pitch_controller::compute(state_params state)
{
	float controllerOutput;

	if (timer.fire())
	{
		previousError = error;
		error = setpoint - state.pitch;

		p_val = float_constrain(p_component(), -(10 * (long)samplePeriod_ms), 10 * (long)samplePeriod_ms); // clip the p-component if it becomes comically large
		i_val = i_component();
		d_val = d_component();

		controllerOutput = p_val + i_val + d_val;
		controllerOutput += roll_compensation(controllerOutput, state);
		controllerOutput = float_constrain(controllerOutput + (outputMin + ((outputMax - outputMin) / 2)), outputMin, outputMax); // add bias (output commands don't have "negative" pulsewidths) and constrain the output

		status = true;
		return controllerOutput;
	}

	status = false;
	return 0;
}




float pitch_controller::roll_compensation(float controllerOutput, state_params state)
{
	return controllerOutput * abs(sin(radians(state.roll)));
}
