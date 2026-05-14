#include "Arm.h"
#include "Camera.h"

int main() {
    Arm arm("/dev/ttyUSB0"); // Update with the correct port for your system
    arm.connect();

    arm.write("cartesian", 0.0, -50.0); // move to rest position on startup

    std::tuple<double, double, double> marker_position = getMarkerPosition(); // will block for 1 second while it captures camera data
    std::cout << "Marker Position (cm): X=" << std::get<0>(marker_position) << ", Y=" << std::get<1>(marker_position) << ", Z=" << std::get<2>(marker_position) << std::endl;
    if (std::get<2>(marker_position) == -1) {
        std::cout << "Failed to detect marker. Exiting." << std::endl;
        return 0;
    }
    
    float camera_offset_x = 4.0; // how far camera is offset from motor 1 output shaft
    float camera_offset_y = 6.7;
    float end_effector_length = 4.0;
    float write_x = std::get<2>(marker_position) + camera_offset_x - end_effector_length;
    float write_y = std::get<1>(marker_position) + camera_offset_y;

    arm.write("cartesian", write_x, write_y);

    return 0;
}