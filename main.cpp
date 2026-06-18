#include "Arm.h"
#include "Camera.h"
#include <csignal>

// global pointer so signal handler can access arm
Arm* g_arm = nullptr;

void signalHandler(int signal) {
    std::cout << "\nShutting down — moving arm to home position..." << std::endl;
    if (g_arm != nullptr) {
        g_arm->non_blocking_state(true);
        g_arm->write("cartesian", 0.0, -50.0, 50, 2);
    }
    std::exit(0);
}

int main() {
    Arm arm("/dev/ttyUSB0");
    arm.connect();

    // register signal handler
    g_arm = &arm;
    std::signal(SIGINT, signalHandler);   // ctrl+c
    std::signal(SIGTERM, signalHandler);  // kill command

    Camera camera;
    camera.open(4);

    double target_x;
    double target_y;

    arm.write("cartesian", 0.0, -50.0, 50, 2);
    arm.non_blocking_state(true);

    while (1) {
        std::tuple<double, double, double> marker_position = camera.getStrawberryPosition();
        target_x = std::get<2>(marker_position);
        target_y = std::get<1>(marker_position);

        std::cout << "Target Position in Camera Coords (cm): X=" << std::get<0>(marker_position) << ", Y=" << std::get<1>(marker_position) << ", Z=" << std::get<2>(marker_position) << std::endl;
        std::cout << "Target Arm Coords (cm): X=" << target_x << ", Y=" << target_y << std::endl;

        if (target_x == -1) {
            std::cout << "Failed to detect marker. Exiting." << std::endl;
            continue;
        }

        target_x = target_x + camera_offset_x - end_effector_length;
        target_y = target_y + camera_offset_y;
        arm.write("cartesian", target_x, target_y, 50, 2);
    }

    return 0;
}