#include "ODrive.h"
#include <iostream>

int main (int argc, char *argv[]) { 
    ODrive::ODrive Hndl;
    Hndl.Reboot(0);
    Hndl.SetState(0, MOTOR_CALIBRATION);
    return 0;
}
