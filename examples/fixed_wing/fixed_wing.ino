#include "SerialTransfer.h"
#include "Autopilot.h"


#define DEBUG_PORT    Serial1 //COM10
#define FEEDBACK_PORT Serial2 //COM8


////////////////////////////////////////////////////////////////////////// Constants
const int JOY_MAX = 1023;
const int JOY_MIN = 0;
const int GEAR_TOGGLE_BUTTON = 1;
const int FLAP_TOGGLE_BUTTON = 2;
const int TAKEOFF_THRESH = 500;


////////////////////////////////////////////////////////////////////////// objects
SerialTransfer feedback;
SerialTransfer datalog;

pitch_controller pitchController;
control_params pitchParams{
  -10,  // setpoint
  10,   // kp
  0,    // ki
  0,    // kd
  10,   // sampleRate
  1023, // outputMax
  0     // ms pulswidth
};

roll_controller rollController;
control_params rollParams{
  0,    // setpoint
  20,   // kp
  0,    // ki
  10,   // kd
  10,   // sampleRate
  1023, // outputMax
  0     // ms pulswidth
};

heading_controller headingController;
control_params headingParams{
  40,   // setpoint
  2,    // kp
  0,    // ki
  0,    // kd
  10,   // sampleRate
  35,   // outputMax
  -35   // outputMin
};

altitude_controller altitudeController;
control_params altitudeParams{
  1000, // setpoint
  0.1,  // kp
  0,    // ki
  0,    // kd
  10,   // sampleRate
  15,   // outputMax
  -15   // outputMin
};

ias_controller iasController;
control_params iasParams{
  350,  // setpoint
  10,   // kp
  0,    // ki
  0,    // kd
  10,   // sampleRate
  1023, // outputMax
  0     // outputMin
};

state_params plane;

struct extra_params{
  float useWaypoints = 0;
  float nextLat      = 0;
  float nextLon      = 0;
  float nextHdg      = 0;
  float nextAlt      = 0;
  float nextIAS      = 0;
} extraParams;

struct inputs{
  float pitch = 0;
  float roll  = 0;
  float hdg   = 0;
  float alt   = 0;
  float ias   = 0;
} setpoints;


uint16_t throttle_command = JOY_MAX * 0.90;
float pitch_command = 512;
float roll_command  = 512;
float yaw_command   = 512;


////////////////////////////////////////////////////////////////////////// setup()
void setup()
{
  DEBUG_PORT.begin(115200);
  FEEDBACK_PORT.begin(115200);
  
  feedback.begin(FEEDBACK_PORT);
  datalog.begin(DEBUG_PORT);

  setupControllers();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
}


////////////////////////////////////////////////////////////////////////// loop()
void loop()
{
  if(handleData())
    handleControllers();
  
  sendJoyCommands();
}


////////////////////////////////////////////////////////////////////////// helping functions
void setupControllers()
{
  pitchController.begin(pitchParams);
  rollController.begin(rollParams);
  headingController.begin(headingParams);
  altitudeController.begin(altitudeParams);
  iasController.begin(iasParams);
}


bool handleData()
{
  if(feedback.available())
  {
    if(feedback.currentPacketID())
    {
      feedback.rxObj(setpoints);

      if(!extraParams.useWaypoints)
      {
        altitudeController.setpoint = setpoints.alt;
        iasController.setpoint      = setpoints.ias;
      }
    }
    else
    {
      feedback.rxObj(extraParams, feedback.rxObj(plane));
      plane.pitch = -plane.pitch;
      plane.roll  = -plane.roll;

      if(extraParams.useWaypoints)
      {
        headingController.setpoint  = find_heading(plane.lat, plane.lon, extraParams.nextLat, extraParams.nextLon);
        altitudeController.setpoint = extraParams.nextAlt;
        iasController.setpoint      = extraParams.nextIAS;
      }
      else
        headingController.setpoint  = setpoints.hdg;

      datalog.sendDatum(setpoints);;
    }

    return true;
  }

  return false;
}


void handleControllers()
{
  float temp_pitch_command = pitchController.compute(plane);
  if(pitchController.status)
    pitch_command = temp_pitch_command;

  float temp_roll_command  = rollController.compute(plane);
  if(rollController.status)
    roll_command = temp_roll_command;

  float temp_heading_command  = -headingController.compute(plane);
  if(headingController.status)
    rollController.setpoint = temp_heading_command;

  float temp_altitude_command  = -altitudeController.compute(plane);
  if(altitudeController.status)
    pitchController.setpoint = temp_altitude_command;

  float temp_ias_command  = iasController.compute(plane);
  if(iasController.status)
    throttle_command = temp_ias_command;

  setpoints.pitch = -pitchController.setpoint;
  setpoints.roll  = -rollController.setpoint;
  setpoints.hdg   = headingController.setpoint;
  setpoints.alt   = altitudeController.setpoint;
  setpoints.ias   = iasController.setpoint;
}


void sendJoyCommands()
{
  Joystick.X(pitch_command);
  Joystick.Y(roll_command);
  Joystick.Z(yaw_command);
  Joystick.Zrotate(throttle_command);
}
