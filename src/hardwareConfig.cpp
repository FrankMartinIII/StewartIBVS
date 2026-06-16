#include "hardwareConfig.h"
//Old vals were good for motor one but bounds are inconsistent across motors
const int minExtensionCommand = 45;
const int maxExtensionCommand = 950;

int convertLengthToCommand(double length)
{
    //int command = (31.91 * (length-121.5)) - 16.01;
    //int command = ((length - 121.5) / 30) * 1023;
    int command = ((length - 122.64) / 30) * 1023;
    return command;
}