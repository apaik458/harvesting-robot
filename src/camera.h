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

// Parameters measured from physical position
constexpr double CameraOffsetX = 6.9;
constexpr double CameraOffsetY = 10.5;

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

  // For aruco (no longer used in main program)
  std::tuple<double, double, double> GetMarkerPosition();

  // For strawberry/stem (currently only strawberry is being used)
  std::tuple<double, double, double> GetStrawberryPosition();
  std::tuple<double, double, double> GetStemPosition();

  void SetFaultInjector(FaultInjector* injector);

  bool IsConnected() const;
  double GetFrameTimestamp() const;
  double GetDepthValue() const;
  bool IsDetectionValid() const;

 private:
  // Shared
  cv::Mat latest_frame_;
  mutable std::mutex frame_mutex_;
  std::thread capture_thread_;
  bool is_open_ = false;

  // Realsense camera
  rs2::pipeline rs_pipeline_;
  rs2::align align_to_color_{RS2_STREAM_COLOR};
  rs2::depth_frame latest_depth_frame_{nullptr};
  mutable std::mutex depth_mutex_;

  // Aruco 
  cv::aruco::Dictionary aruco_dict_;
  cv::aruco::DetectorParameters aruco_params_;
  cv::aruco::ArucoDetector aruco_detector_;
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;
  const float MarkerSizeCm = 3.85f;

  // Yolov5
  cv::dnn::Net yolo_net_;
  const float ConfThreshold = 0.5f;
  const int YoloInputSize = 640;
  const int ClassStrawberry = 1;
  const int ClassStem = 0;

  std::vector<Detection> latest_detections_;
  mutable std::mutex detections_mutex_;

  // Helpers
  std::vector<Detection> RunYolo(const cv::Mat& frame);
  float GetMedianDepth(const rs2::depth_frame& depth, float x1, float y1, float x2, float y2);
  std::tuple<double, double, double> DeprojectToWorld(const rs2::depth_frame& depth, float cx, float cy);

  FaultInjector* fault_injector_ = nullptr;
  double frame_timestamp_ = 0.0;
  double depth_value_ = -1.0;
};
