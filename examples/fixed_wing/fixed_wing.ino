#include "SerialTransfer.h"
#include "Autopilot.h"


#define DEBUG_PORT    Serial2 //COM5
#define FEEDBACK_PORT Serial5 //COM4

const byte THROTTLE_PIN = 17;
const byte PITCH_PIN    = 36;
const byte ROLL_PIN     = 14;
const byte YAW_PIN      = 16;

int JOY_MAX = 1023;
int JOY_MIN = 0;

const unsigned int THROTTLE_MAX_ADC = 41060;
const unsigned int THROTTLE_MIN_ADC = 24130;
const unsigned int PITCH_MAX_ADC    = 41220;
const unsigned int PITCH_MIN_ADC    = 23700;
const unsigned int ROLL_MAX_ADC     = 41900;
const unsigned int ROLL_MIN_ADC     = 25000;
const unsigned int YAW_MAX_ADC      = 41700;
const unsigned int YAW_MIN_ADC      = 24900;


SerialTransfer feedback;
pitch_controller elevator;


float pitch = 0;
float roll  = 0;
float ias   = 0;

uint16_t pilot_throttle = 0;
uint16_t pilot_pitch    = 512;
uint16_t pilot_roll     = 512;
uint16_t pilot_yaw      = 512;


////////////////////////////////////////////////////////////////////////// setup()
void setup()
{
  DEBUG_PORT.begin(115200);
  FEEDBACK_PORT.begin(115200);

  feedback.begin(FEEDBACK_PORT);
  elevator.begin(0.0,  // setpoint
                 1,    // max rate up
                 1,    // max rate down
                 0.05, // kp
                 0.0,  // ki
                 0.0,  // kd
                 0.7,  // roll compensation
                 20.0, // sample rate
                 JOY_MAX,  // servo max
                 JOY_MIN); // servo min

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  analogReadResolution(16);
}


////////////////////////////////////////////////////////////////////////// loop()
void loop()
{
  getUserInputs();
  
  if(getFeedback())
  {
    pilot_pitch = elevator.compute(pitch, roll, ias);
  }

  sendJoyCommands();
}


void getUserInputs()
{
  pilot_throttle = constrain(map(analogRead(THROTTLE_PIN), THROTTLE_MIN_ADC, THROTTLE_MAX_ADC, JOY_MAX, JOY_MIN), JOY_MIN, JOY_MAX);
  //pilot_pitch    = constrain(map(analogRead(PITCH_PIN),    PITCH_MIN_ADC,    PITCH_MAX_ADC,    JOY_MIN, JOY_MAX), JOY_MIN, JOY_MAX);
  pilot_roll     = constrain(map(analogRead(ROLL_PIN),     ROLL_MIN_ADC,     ROLL_MAX_ADC,     JOY_MAX, JOY_MIN), JOY_MIN, JOY_MAX);
  pilot_yaw      = constrain(map(analogRead(YAW_PIN),      YAW_MIN_ADC,      YAW_MAX_ADC,      JOY_MIN, JOY_MAX), JOY_MIN, JOY_MAX);
}


bool getFeedback()
{
  if(feedback.available() && (feedback.bytesRead >= 6))
  {
    pitch = ((int16_t)(feedback.rxBuff[0] << 8) | feedback.rxBuff[1]) / 350.0;
    roll  = ((int16_t)(feedback.rxBuff[2] << 8) | feedback.rxBuff[3]) / 350.0;
    ias   = ((int16_t)(feedback.rxBuff[4] << 8) | feedback.rxBuff[5]) / 1000.0;

    return true;
  }
  else if(feedback.status < 0)
  {
    DEBUG_PORT.print("ERROR: ");
    DEBUG_PORT.println(feedback.status);
  }

  return false;
}


void sendJoyCommands()
{
  Joystick.X(pilot_pitch);          //roll
  Joystick.Y(pilot_roll);           //pitch
  Joystick.Z(pilot_yaw);            //yaw
  Joystick.Zrotate(pilot_throttle); //throttle
  Joystick.hat(-1);                 //hat
}
