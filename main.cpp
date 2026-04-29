#include "Arm.h"

int main() {
    Arm arm("/dev/ttyUSB0"); // Update with the correct port for your system

    arm.connect();
    arm.write(2048, 2048, 2048); // 2048 is the middle position for a Dynamixel with a range of 0-4095

    return 0;
}