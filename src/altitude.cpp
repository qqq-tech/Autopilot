#include "Autopilot.h"




float altitude_controller::compute(const state_params& state)
{
	float controllerOutput;

	if (timer.fire())
	{
		previousError = error;
		error = setpoint - state.alt;

		p_val = constrain(p_component(), -(10 * (long)samplePeriod_ms), 10 * (long)samplePeriod_ms);
		i_val = i_component();
		d_val = d_component();

		controllerOutput = p_val + i_val + d_val;
		controllerOutput = float_constrain(controllerOutput + (outputMin + ((outputMax - outputMin) / 2)), outputMin, outputMax); // add bias (output commands don't have "negative" pulsewidths) and constrain the output

		status = true;
		return controllerOutput;
	}

	status = false;
	return 0;
}
