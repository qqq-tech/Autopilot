#include "Autopilot.h"




float pitch_controller::compute(const state_params& state)
{
	float controllerOutput;

	if (timer.fire())
	{
		previousError = error;
		error = setpoint - state.pitch;

		p_val = p_component();
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




float pitch_controller::compute(const pilsim_state_params& state)
{
	float controllerOutput;

	if (timer.fire())
	{
		previousError = error;
		error = setpoint - state.pitch;

		p_val = p_component();
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




float pitch_controller::roll_compensation(const float& controllerOutput, const state_params& state)
{
	return controllerOutput * abs(sin(radians(state.roll)));
}




float pitch_controller::roll_compensation(const float& controllerOutput, const pilsim_state_params& state)
{
	return controllerOutput * abs(sin(radians(state.roll)));
}
