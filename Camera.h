#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/dnn.hpp>
#include <librealsense2/rs.hpp>
#include <tuple>
#include <chrono>
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>
#include <unistd.h>

#define camera_offset_x 6.9
#define camera_offset_y 10.5

struct Detection {
    float x1, y1, x2, y2;
    int class_id;
    float confidence;
};

class Camera {
public:
    Camera();
    ~Camera();
    bool open(int device_id = 4);

    // aruco
    std::tuple<double, double, double> getMarkerPosition();

    // strawberry/stem
    std::tuple<double, double, double> getStrawberryPosition();
    std::tuple<double, double, double> getStemPosition();

private:
    // shared
    cv::Mat latest_frame;
    std::mutex frame_mutex;
    std::thread capture_thread;
    bool is_open = false;

    // realsense
    rs2::pipeline rs_pipeline;
    rs2::align align_to_color{RS2_STREAM_COLOR};
    rs2::depth_frame latest_depth_frame{nullptr};
    std::mutex depth_mutex;

    // aruco — new API
    cv::aruco::Dictionary aruco_dict;
    cv::aruco::DetectorParameters aruco_params;
    cv::aruco::ArucoDetector aruco_detector;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    const float MARKER_SIZE_CM = 3.85f;

    // yolo
    cv::dnn::Net yolo_net;
    const float CONF_THRESHOLD = 0.5f;
    const int YOLO_INPUT_SIZE = 640;
    const int CLASS_STRAWBERRY = 1;
    const int CLASS_STEM = 0;

    // helpers
    std::vector<Detection> runYOLO(const cv::Mat& frame);
    float getMedianDepth(const rs2::depth_frame& depth, float x1, float y1, float x2, float y2);
    std::tuple<double, double, double> deprojectToWorld(const rs2::depth_frame& depth, float cx, float cy);
};