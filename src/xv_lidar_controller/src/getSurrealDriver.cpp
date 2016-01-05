#include "xv_lidar_controller/getSurrealDriver.h"

// CONFIGURATION
static const int BAUDRATE = 115200;
static const int TIMEOUT = 1000;
static const string PORTNAME = "/dev/ttyACM0";

// GETSURREAL API COMMANDS
static const string CMD_HELP = "Help";
static const string CMD_CONFIG_SHOW = "ShowConfig";
static const string CMD_CONFIG_SAVE = "SaveConfig";
static const string CMD_CONFIG_RESET = "ResetConfig";
static const string CMD_SET_RPM = "SetRPM";
static const string CMD_SET_KP = "SetKp";
static const string CMD_SET_KI = "SetKi";
static const string CMD_SET_KD = "SetKd";
static const string CMD_SET_SAMPLETIME = "SetSampleTime";
static const string CMD_SHOW_RPM = "ShowRPM";
static const string CMD_HIDE_RPM = "HideRPM";
static const string CMD_SHOW_DISTANCES = "ShowDist";
static const string CMD_HIDE_DISTANCES = "HideDist";
static const string CMD_SHOW_ANGLE = "ShowAngle";
static const string CMD_MOTOR_OFF = "MotorOff";
static const string CMD_MOTOR_ON = "MotorOn";
static const string CMD_HIDE_RAW = "HideRaw";
static const string CMD_SHOW_RAW = "ShowRaw";

GetSurrealDriver::GetSurrealDriver() {
  serial::Serial my_serial(PORTNAME, BAUDRATE, serial::Timeout::simpleTimeout(TIMEOUT));
}

int main()
{
  return 0;
}
