#include "serial/serial.h"

class GetSurrealDriver {
    serial::Serial serialCon;
  public:
    GetSurrealDriver();
    virtual ~GetSurrealDriver();
    void motorOn();
    void motorOff();
    bool readNextValues(int *angle, double *range, double *intensity);
};
