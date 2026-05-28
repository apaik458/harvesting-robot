#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tuple>
#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>

#define camera_offset_x 6.9 // how far camera is offset from motor 1 output shaft
#define camera_offset_y 10.5

class Camera {
public:
    Camera();
    ~Camera();

    bool open(int device_id = 4);
    std::tuple<double, double, double> getMarkerPosition();

private:
    cv::VideoCapture cap;
    cv::Ptr<cv::aruco::Dictionary> aruco_dict;
    cv::Ptr<cv::aruco::DetectorParameters> aruco_params;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    const float MARKER_SIZE_CM = 3.85f;
    bool is_open = false;

    cv::Mat latest_frame;
    std::mutex frame_mutex;
    std::thread capture_thread;
};