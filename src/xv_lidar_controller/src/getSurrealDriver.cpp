#include <string>
#include <xv_lidar_controller/getSurrealDriver.h>
#include <iostream>
#include <cstdio>
#include <cstdlib>

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

static const string TIME = "Time";
static const string RPM = "RPM";

GetSurrealDriver::GetSurrealDriver() : serialCon(PORTNAME, BAUDRATE, serial::Timeout::simpleTimeout(TIMEOUT)) {
}
GetSurrealDriver::~GetSurrealDriver() {};


void GetSurrealDriver::motorOn() {
  if(serialCon.isOpen()) {
    serialCon.write(CMD_MOTOR_ON);
  }  
}

void GetSurrealDriver::motorOff() {
  if(serialCon.isOpen()) {
    serialCon.write(CMD_MOTOR_OFF);
  }
}

bool GetSurrealDriver::readNextValues(int *angle, double *range, double *intensity) {
    if(serialCon.isOpen()) {
        // Read data from serial interface
        string data = serialCon.readline();
        // If data contains either the Word Time or RPM return false
        std::size_t found = data.find(TIME);
        if (found!=std::string::npos) return false;
        found = data.find(RPM);
        if (found!=std::string::npos) return false;

        // Get the angle value
        found = data.find(":");
        std::istringstream(data.substr(0, found)) >> *angle;

        // Get the range value
        std::size_t start = found+2;
        found = data.find("(", start);
        double range_mm = std::atof(data.substr(start, found-1).c_str());
        *range = range_mm / 1000.0;

        // Get the intensity value
        start = found+1;
        found = data.find(")", start);
        *intensity = std::atof(data.substr(start, found).c_str());
        
        return true;
    }
    return false;
}












