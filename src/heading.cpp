#include "Autopilot.h"




float heading_controller::compute(const state_params& state)
{
	float controllerOutput;

	if (timer.fire())
	{
		previousError = error;
		error = setpoint - inputScrub(state);

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




float heading_controller::inputScrub(const state_params& state)
{
	float decisionPoint = fmod((state.hdg + 180), 360);

	if (setpoint > 180)
	{
		if (state.hdg < decisionPoint)
			return state.hdg + 360;
		
		return state.hdg;
	}
	else if ((setpoint < 180) && (setpoint != 0))
	{
		if (state.hdg > decisionPoint)
			return state.hdg - 360;
		
		return state.hdg;
	}
	else if (setpoint == 180)
	{
		if (state.hdg == decisionPoint)
			return 0.01;
		
		return state.hdg;
	}
	else if (setpoint == 0)
	{
		if (state.hdg > decisionPoint)
			return state.hdg - 360;

		return state.hdg;
	}
	
	return state.hdg;
}
