#include "serial/serial.h"

class GetSurrealDriver {
    serial::Serial serialCon;
  public:
    GetSurrealDriver();
    virtual ~GetSurrealDriver();
    void MotorOn();
    void MotorOff();
};
