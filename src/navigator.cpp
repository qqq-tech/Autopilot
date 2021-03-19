#include "Autopilot.h"




void navigator::processFrame()
{
	findMTR(navFrame.ni);
	findMTR(navFrame.nf);

	findTurnCenters(navFrame.ni);
	findTurnCenters(navFrame.nf);

	findPath(navFrame);

	findEPts(navFrame);
}




void navigator::processFrame(nav_frame& frame)
{
	findHAR(frame.ni);
	findHAR(frame.nf);

	findMTR(frame.ni);
	findMTR(frame.nf);

	findTurnCenters(frame.ni);
	findTurnCenters(frame.nf);

	findPath(frame);

	findEPts(frame);
}




void navigator::findMTR(point& curPoint)
{
	curPoint.minTurnRad = (curPoint.speed * curPoint.speed) / (EARTH_GRAVITY * tan(radians(curPoint.maxRoll)));
}




void navigator::findTurnCenters(point& curPoint)
{
	coord(curPoint.lat, curPoint.lon, curPoint.rc_lat, curPoint.rc_lon, curPoint.minTurnRad, fmod((curPoint.heading + 90), 360));
	coord(curPoint.lat, curPoint.lon, curPoint.lc_lat, curPoint.lc_lon, curPoint.minTurnRad, fmod((curPoint.heading - 90) + 360, 360));
}




void navigator::findPath(nav_frame& frame)
{
	float dist_ir = distance(frame.ni.rc_lat, frame.ni.rc_lon, frame.nf.lat, frame.nf.lon);
	float dist_il = distance(frame.ni.lc_lat, frame.ni.lc_lon, frame.nf.lat, frame.nf.lon);

	if (dist_ir <= dist_il)
	{
		frame.ni.c_lat = frame.ni.rc_lat;
		frame.ni.c_lon = frame.ni.rc_lon;
	}
	else
	{
		frame.ni.c_lat = frame.ni.lc_lat;
		frame.ni.c_lon = frame.ni.lc_lon;
	}

	float dist_fr = distance(frame.nf.rc_lat, frame.nf.rc_lon, frame.ni.lat, frame.ni.lon);
	float dist_fl = distance(frame.nf.lc_lat, frame.nf.lc_lon, frame.ni.lat, frame.ni.lon);

	if (dist_fr <= dist_fl)
	{
		frame.nf.c_lat = frame.nf.rc_lat;
		frame.nf.c_lon = frame.nf.rc_lon;
	}
	else
	{
		frame.nf.c_lat = frame.nf.lc_lat;
		frame.nf.c_lon = frame.nf.lc_lon;
	}

	if (dist_ir <= dist_il)
	{
		if (dist_fr <= dist_fl)
		{
			if (frame.nf.alt >= frame.ni.alt)
				frame.path = RSRU;
			else
				frame.path = RSRD;
		}
		else
		{
			if (frame.nf.alt >= frame.ni.alt)
				frame.path = RSLU;
			else
				frame.path = RSLD;
		}
	}
	else
	{
		if (dist_fr <= dist_fl)
		{
			if (frame.nf.alt >= frame.ni.alt)
				frame.path = LSRU;
			else
				frame.path = LSRD;
		}
		else
		{
			if (frame.nf.alt >= frame.ni.alt)
				frame.path = LSLU;
			else
				frame.path = LSLD;
		}
	}
}




void navigator::findEPts(nav_frame& frame)
{
	float xi, yi, xf, yf, cxi, cyi, cxf, cyf;

	toXY(frame.ni.lat,   frame.ni.lon,   xi,  yi,  frame.ni.lat, frame.ni.lon);
	toXY(frame.ni.c_lat, frame.ni.c_lon, cxi, cyi, frame.ni.lat, frame.ni.lon);
	toXY(frame.nf.lat,   frame.nf.lon,   xf,  yf,  frame.ni.lat, frame.ni.lon);
	toXY(frame.nf.c_lat, frame.nf.c_lon, cxf, cyf, frame.ni.lat, frame.ni.lon);

	if ((frame.path == LSRU) || (frame.path == LSRD))
	{

	}
	else if ((frame.path == RSLU) || (frame.path == RSLD))
	{

	}
	else if ((frame.path == RSRU) || (frame.path == RSRD))
	{

	}
	else if ((frame.path == LSLU) || (frame.path == LSLD))
	{

	}
}
