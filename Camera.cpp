#include "Camera.h"

Camera::Camera() {
    aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    aruco_params = cv::aruco::DetectorParameters::create();
    camera_matrix = (cv::Mat_<double>(3, 3) <<
        603.5139681293208, 0.0, 322.4871143178484,
        0.0, 606.2351953521359, 238.09114115221269, // these values have been hand entered after the camera calibration process in "camera_calibration"
        0.0, 0.0, 1.0);
    dist_coeffs = (cv::Mat_<double>(1, 5) <<
        -0.005862800130731961, 1.193475628221078,
        0.0006140022848452339, -0.0003950816468824019, -4.37080936712476);
}

Camera::~Camera() {
    if (is_open) {
        is_open = false;
        if (capture_thread.joinable()) capture_thread.join();
        cap.release();
    }
}

bool Camera::open(int device_id) {
    cap.open(device_id, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open camera on /dev/video" << device_id << std::endl;
        return false;
    }
    is_open = true;
    std::cout << "Camera opened on /dev/video" << device_id << std::endl;

    // start capture thread
    capture_thread = std::thread([this]() {
    while (is_open) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) continue;

        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            latest_frame = frame.clone();
        }

        // draw detected markers on preview
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> rejected;
        cv::aruco::detectMarkers(frame, aruco_dict, corners, ids, aruco_params, rejected);
        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
        }

        cv::imshow("Camera Preview", frame);
        cv::waitKey(1); // needed to actually render the window
    }
    cv::destroyWindow("Camera Preview");
    });

    // wait for first frame to arrive before returning
    std::cout << "Waiting for first frame..." << std::endl;
    while (true) {
        std::lock_guard<std::mutex> lock(frame_mutex);
        if (!latest_frame.empty()) break;
    }
    std::cout << "Camera ready." << std::endl;

    return true;
}

std::tuple<double, double, double> Camera::getMarkerPosition() {
    if (!is_open) {
        std::cerr << "Error: camera not opened. Call open() first." << std::endl;
        return {-1, -1, -1};
    }

    // auto start = std::chrono::steady_clock::now();

    // grab latest frame from the shared buffer
    cv::Mat frame;
    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        frame = latest_frame.clone();
    }

    std::tuple<double, double, double> result = {-1, -1, -1};

    if (!frame.empty()) {
        std::vector<std::vector<cv::Point2f>> corners;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> rejected;
        cv::aruco::detectMarkers(frame, aruco_dict, corners, ids, aruco_params, rejected);

        if (!ids.empty()) {
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(
                corners, MARKER_SIZE_CM, camera_matrix, dist_coeffs, rvecs, tvecs
            );
            result = {tvecs[0][0], -tvecs[0][1], tvecs[0][2]};
        }
    }

    // wait out remainder of 250ms
    // auto elapsed = std::chrono::steady_clock::now() - start;
    // auto remaining = std::chrono::milliseconds(250) - elapsed;
    // if (remaining > std::chrono::milliseconds(0)) {
    //     std::this_thread::sleep_for(remaining);
    // }

    return result;
}