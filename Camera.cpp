#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tuple>
#include <iostream>
#include <chrono>

std::tuple<double, double, double> getMarkerPosition() {
    cv::Ptr<cv::aruco::Dictionary> aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params = cv::aruco::DetectorParameters::create();

    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
        603.5139681293208, 0.0, 322.4871143178484,
        0.0, 606.2351953521359, 238.09114115221269,
        0.0, 0.0, 1.0);
    cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) <<
        -0.005862800130731961, 1.193475628221078,
        0.0006140022848452339, -0.0003950816468824019, -4.37080936712476);

    const float MARKER_SIZE_CM = 3.85f;

    cv::VideoCapture cap(4, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open camera" << std::endl;
        return {-1, -1, -1};
    }

    double last_x = -1, last_y = -1, last_z = -1;
    auto start = std::chrono::steady_clock::now();

    cv::Mat frame;
    while (true) {
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() >= 1000) {
            break;
        }

        cap >> frame;
        if (frame.empty()) continue;

        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> rejected;
        cv::aruco::detectMarkers(frame, aruco_dict, corners, ids, aruco_params, rejected);

        if (!ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(
                corners, MARKER_SIZE_CM, camera_matrix, dist_coeffs, rvecs, tvecs
            );
            last_x = tvecs[0][0];
            last_y = -tvecs[0][1];
            last_z = tvecs[0][2];
        }
    }

    cap.release();
    return {last_x, last_y, last_z};
}