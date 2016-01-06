#include <string>
#include <xv_lidar_controller/getSurrealDriver.h>
#include <iostream>
#include <cstdio>

using std::string;
using std::cout;

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
static const string CMD_MOTOR_OFF = "MotorOff\n";
static const string CMD_MOTOR_ON = "MotorOn\n";
static const string CMD_HIDE_RAW = "HideRaw";
static const string CMD_SHOW_RAW = "ShowRaw";

GetSurrealDriver::GetSurrealDriver() : serialCon(PORTNAME, BAUDRATE, serial::Timeout::simpleTimeout(TIMEOUT)) {
}
GetSurrealDriver::~GetSurrealDriver() {};


void GetSurrealDriver::MotorOn() {
  cout << "Is the serial port open?";
  if(serialCon.isOpen())
    cout << " Yes." << std::endl;
  else
    cout << " No." << std::endl;
  string cmdOn = CMD_MOTOR_ON;
  serialCon.write(cmdOn);
}

void GetSurrealDriver::MotorOff() {
  string cmdOff = CMD_MOTOR_OFF;
  serialCon.write(cmdOff);
}
