#include "Arm.h"

int main() {
    Arm arm("/dev/ttyUSB0"); // Update with the correct port for your system

    arm.connect();
    arm.write("cartesian", 0.0, 50.0);   

    return 0;
}