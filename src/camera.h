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
#include "fault_injector.h"

constexpr double kCameraOffsetX = 6.9;
constexpr double kCameraOffsetY = 10.5;

struct Detection {
  float x1, y1, x2, y2;
  int class_id;
  float confidence;
};

class Camera {
 public:
  Camera();
  ~Camera();
  bool Open(int device_id = 4);

  // aruco
  std::tuple<double, double, double> GetMarkerPosition();

  // strawberry/stem
  std::tuple<double, double, double> GetStrawberryPosition();
  std::tuple<double, double, double> GetStemPosition();

  // --- Safety / HITL fault testing ---
  // Injects fault scenarios at Camera's raw hardware read points (the
  // capture thread). Pass nullptr (the default) for normal operation.
  void SetFaultInjector(FaultInjector* injector) { fault_injector_ = injector; }

  bool IsConnected() const;
  double GetFrameTimestamp() const;
  double GetDepthValue() const;
  bool IsDetectionValid() const;

 private:
  // shared
  cv::Mat latest_frame_;
  mutable std::mutex frame_mutex_;
  std::thread capture_thread_;
  bool is_open_ = false;

  // realsense
  rs2::pipeline rs_pipeline_;
  rs2::align align_to_color_{RS2_STREAM_COLOR};
  rs2::depth_frame latest_depth_frame_{nullptr};
  mutable std::mutex depth_mutex_;

  // aruco — new API
  cv::aruco::Dictionary aruco_dict_;
  cv::aruco::DetectorParameters aruco_params_;
  cv::aruco::ArucoDetector aruco_detector_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  const float kMarkerSizeCm = 3.85f;

  // yolo
  cv::dnn::Net yolo_net_;
  const float kConfThreshold = 0.5f;
  const int kYoloInputSize = 640;
  const int kClassStrawberry = 1;
  const int kClassStem = 0;

  std::vector<Detection> latest_detections_;
  mutable std::mutex detections_mutex_;

  // helpers
  std::vector<Detection> RunYolo(const cv::Mat& frame);
  float GetMedianDepth(const rs2::depth_frame& depth, float x1, float y1, float x2, float y2);
  std::tuple<double, double, double> DeprojectToWorld(const rs2::depth_frame& depth, float cx, float cy);

  // --- Safety / HITL fault testing ---
  FaultInjector* fault_injector_ = nullptr;
  double frame_timestamp_ = 0.0;  // guarded by frame_mutex_
  double depth_value_ = -1.0;     // guarded by depth_mutex_, metres at frame centre
};
