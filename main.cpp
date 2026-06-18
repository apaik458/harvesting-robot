#include "Arm.h"
#include "Camera.h"

int main() {
    Arm arm("/dev/ttyUSB0"); // Update with the correct port for your system
    arm.connect();
    Camera camera;
    camera.open(4); // using the "/dev/video4" camera stream
    
    double target_x;
    double target_y;

    arm.write("cartesian", 0.0, -50.0, 50, 2); // Starting arm at home position
    arm.toggle_blocking_state(true); // After homing is complete, want the arm movement to be quick and non-blocking for reactive target tracking

    while (1) {
        std::tuple<double, double, double> marker_position = camera.getStrawberryPosition(); // will block while it captures camera data
        target_x = std::get<2>(marker_position); // the camera and arm coordinate systems are different; converting to the arm's x-y coordinate system
        target_y = std::get<1>(marker_position);

        std::cout << "Target Position in Camera Coords (cm): X=" << std::get<0>(marker_position) << ", Y=" << std::get<1>(marker_position) << ", Z=" << std::get<2>(marker_position) << std::endl;
        std::cout << "Target Arm Coords (cm): X=" << target_x << ", Y=" << target_y << std::endl;

        if (target_x == -1) {
            std::cout << "Failed to detect marker. Exiting." << std::endl;
            continue;
        }

        target_x = target_x + camera_offset_x - end_effector_length; // accounting for how camera/end-effector are mounted
        target_y = target_y + camera_offset_y;

        arm.write("cartesian", target_x, target_y, 50, 2);
    }
    return 0;
}