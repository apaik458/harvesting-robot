#include "camera.h"

Camera::Camera() {
  // aruco setup - new API
  aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
  aruco_params_ = cv::aruco::DetectorParameters();
  aruco_detector_ = cv::aruco::ArucoDetector(aruco_dict_, aruco_params_);

  camera_matrix_ = (cv::Mat_<double>(3, 3) <<
      603.5139681293208, 0.0, 322.4871143178484,
      0.0, 606.2351953521359, 238.09114115221269,
      0.0, 0.0, 1.0);
  dist_coeffs_ = (cv::Mat_<double>(1, 5) <<
      -0.005862800130731961, 1.193475628221078,
      0.0006140022848452339, -0.0003950816468824019, -4.37080936712476);

  // load yolo onnx model
  char exe_path[1024];
  ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
  exe_path[len] = '\0';
  std::string exe_dir = std::string(exe_path).substr(0, std::string(exe_path).find_last_of("/"));
  std::string model_path = exe_dir + "/../machine_learning/model_output/weights/best.onnx";
  yolo_net_ = cv::dnn::readNetFromONNX(model_path);
  if (yolo_net_.empty()) {
    std::cerr << "Error: could not load YOLO model from " << model_path << std::endl;
  } else {
    yolo_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    yolo_net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    std::cout << "YOLO model loaded." << std::endl;
  }
}

Camera::~Camera() {
  if (is_open_) {
    is_open_ = false;
    if (capture_thread_.joinable()) capture_thread_.join();
    rs_pipeline_.stop();
  }
}

bool Camera::Open(int device_id) {
  // start realsense pipeline with colour and depth streams
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

  try {
    rs_pipeline_.start(cfg);
  } catch (const rs2::error& e) {
    std::cerr << "Error: could not open RealSense camera: " << e.what() << std::endl;
    return false;
  }

  is_open_ = true;
  std::cout << "RealSense camera opened." << std::endl;

  // start capture thread
  capture_thread_ = std::thread([this]() {
    while (is_open_) {
      rs2::frameset frames = rs_pipeline_.wait_for_frames();

      // align depth to colour
      rs2::frameset aligned = align_to_color_.process(frames);
      rs2::video_frame color_frame = aligned.get_color_frame();
      rs2::depth_frame depth_frame = aligned.get_depth_frame();

      if (!color_frame || !depth_frame) continue;

      // convert colour frame to cv::Mat
      cv::Mat frame(
          cv::Size(color_frame.get_width(), color_frame.get_height()),
          CV_8UC3,
          (void*)color_frame.get_data(),
          cv::Mat::AUTO_STEP
      );

      // Raw hardware read point: frame timestamp. Not consumed by
      // FaultMonitor yet — kept cached here for when a frame-freeze
      // fault check is added back.
      double raw_timestamp = color_frame.get_timestamp();

      // Raw hardware read point: depth at the frame centre. Not consumed
      // by FaultMonitor yet — kept cached here for when a depth-invalid
      // fault check is added back.
      double raw_depth = depth_frame.get_distance(color_frame.get_width() / 2, color_frame.get_height() / 2);

      // store latest frame and depth
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_ = frame.clone();
        frame_timestamp_ = raw_timestamp;
      }
      {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        latest_depth_frame_ = depth_frame;
        depth_value_ = raw_depth;
      }

      // draw aruco markers
      std::vector<std::vector<cv::Point2f>> corners;
      std::vector<int> ids;
      std::vector<std::vector<cv::Point2f>> rejected;
      aruco_detector_.detectMarkers(frame, corners, ids, rejected);
      if (!ids.empty()) {
        cv::aruco::drawDetectedMarkers(frame, corners, ids);
      }

      // draw latest yolo detections
      cv::Mat display = frame.clone();
      {
        std::lock_guard<std::mutex> lock(detections_mutex_);
        for (auto& det : latest_detections_) {
          cv::Scalar colour = (det.class_id == kClassStrawberry)
              ? cv::Scalar(0, 0, 255)
              : cv::Scalar(0, 255, 0);
          cv::rectangle(display, cv::Point(det.x1, det.y1), cv::Point(det.x2, det.y2), colour, 2);
          std::string label = std::string(det.class_id == kClassStrawberry ? "strawberry" : "stem")
              + " " + std::to_string((int)(det.confidence * 100)) + "%";
          cv::putText(display, label, cv::Point(det.x1, det.y1 - 5),
              cv::FONT_HERSHEY_SIMPLEX, 0.5, colour, 1);
          float cx = (det.x1 + det.x2) / 2;
          float cy = (det.y1 + det.y2) / 2;
          cv::circle(display, cv::Point(cx, cy), 4, colour, -1);
        }
      }

      cv::imshow("Camera Preview", display);
      cv::waitKey(1);
    }
    cv::destroyWindow("Camera Preview");
  });

  // wait for first frame
  std::cout << "Waiting for first frame..." << std::endl;
  while (true) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!latest_frame_.empty()) break;
  }
  std::cout << "Camera ready." << std::endl;
  return true;
}

// ─── ArUco ───────────────────────────────────────────────────────────────────

std::tuple<double, double, double> Camera::GetMarkerPosition() {
  if (!is_open_) {
    std::cerr << "Error: camera not opened. Call Open() first." << std::endl;
    return {-1, -1, -1};
  }

  cv::Mat frame;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    frame = latest_frame_.clone();
  }

  std::tuple<double, double, double> result = {-1, -1, -1};
  if (!frame.empty()) {
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> rejected;

    // new API - use detector object
    aruco_detector_.detectMarkers(frame, corners, ids, rejected);

    if (!ids.empty()) {
      // new API - estimatePoseSingleMarkers is replaced
      std::vector<cv::Vec3d> rvecs(ids.size()), tvecs(ids.size());
      for (size_t i = 0; i < ids.size(); i++) {
        cv::solvePnP(
            std::vector<cv::Point3f>{
                {-kMarkerSizeCm / 2,  kMarkerSizeCm / 2, 0},
                { kMarkerSizeCm / 2,  kMarkerSizeCm / 2, 0},
                { kMarkerSizeCm / 2, -kMarkerSizeCm / 2, 0},
                {-kMarkerSizeCm / 2, -kMarkerSizeCm / 2, 0}
            },
            corners[i],
            camera_matrix_,
            dist_coeffs_,
            rvecs[i],
            tvecs[i]
        );
      }
      result = {tvecs[0][0], -tvecs[0][1], tvecs[0][2]};
    }
  }
  return result;
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

float Camera::GetMedianDepth(const rs2::depth_frame& depth, float x1, float y1, float x2, float y2) {
  float shrink_x = (x2 - x1) * 0.25f;
  float shrink_y = (y2 - y1) * 0.25f;
  int inner_x1 = (int)(x1 + shrink_x);
  int inner_x2 = (int)(x2 - shrink_x);
  int inner_y1 = (int)(y1 + shrink_y);
  int inner_y2 = (int)(y2 - shrink_y);

  std::vector<float> depths;
  for (int x = inner_x1; x <= inner_x2; x++) {
    for (int y = inner_y1; y <= inner_y2; y++) {
      float d = depth.get_distance(x, y);
      if (d > 0.0f) depths.push_back(d);
    }
  }

  if (depths.empty()) return -1.0f;

  std::sort(depths.begin(), depths.end());
  return depths[depths.size() / 2];
}

std::tuple<double, double, double> Camera::DeprojectToWorld(const rs2::depth_frame& depth, float cx, float cy) {
  rs2_intrinsics intr = depth.get_profile()
                            .as<rs2::video_stream_profile>()
                            .get_intrinsics();

  float d = depth.get_distance((int)cx, (int)cy);
  if (d <= 0.0f) return {-1, -1, -1};

  float pixel[2] = {cx, cy};
  float point[3];
  rs2_deproject_pixel_to_point(point, &intr, pixel, d);

  // convert metres to cm to match ArUco and arm coordinate system
  return {point[0] * 100.0, -point[1] * 100.0, point[2] * 100.0};
}

// ─── YOLO inference helper ────────────────────────────────────────────────────

std::vector<Detection> Camera::RunYolo(const cv::Mat& frame) {
  std::vector<Detection> detections;

  float scale_x = (float)frame.cols / kYoloInputSize;
  float scale_y = (float)frame.rows / kYoloInputSize;

  cv::Mat blob = cv::dnn::blobFromImage(
      frame, 1.0 / 255.0,
      cv::Size(kYoloInputSize, kYoloInputSize),
      cv::Scalar(), true, false
  );

  yolo_net_.setInput(blob);
  cv::Mat output = yolo_net_.forward();

  // YOLOv5 output shape: [1, 25200, 7]
  cv::Mat output_2d = output.reshape(1, output.size[1]);

  for (int i = 0; i < output_2d.rows; i++) {
    float* row = output_2d.ptr<float>(i);

    float cx         = row[0];
    float cy         = row[1];
    float w          = row[2];
    float h          = row[3];
    float objectness = row[4];
    float conf_stem        = row[5] * objectness;
    float conf_strawberry  = row[6] * objectness;

    float confidence;
    int class_id;
    if (conf_strawberry > conf_stem) {
      confidence = conf_strawberry;
      class_id = kClassStrawberry;
    } else {
      confidence = conf_stem;
      class_id = kClassStem;
    }

    if (confidence < kConfThreshold) continue;

    Detection d;
    d.x1 = (cx - w / 2) * scale_x;
    d.y1 = (cy - h / 2) * scale_y;
    d.x2 = (cx + w / 2) * scale_x;
    d.y2 = (cy + h / 2) * scale_y;
    d.class_id = class_id;
    d.confidence = confidence;
    detections.push_back(d);
  }

  return detections;
}

// ─── Strawberry position ──────────────────────────────────────────────────────

std::tuple<double, double, double> Camera::GetStrawberryPosition() {
  if (!is_open_) {
    std::cerr << "Error: camera not opened. Call Open() first." << std::endl;
    return {-1, -1, -1};
  }

  cv::Mat frame;
  rs2::depth_frame depth{nullptr};
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    frame = latest_frame_.clone();
  }
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    depth = latest_depth_frame_;
  }

  if (frame.empty() || !depth) return {-1, -1, -1};

  auto detections = RunYolo(frame);
  {
    std::lock_guard<std::mutex> lock(detections_mutex_);
    latest_detections_ = detections;
  }

  for (auto& det : detections) {
    if (det.class_id != kClassStrawberry) continue;

    float cx = (det.x1 + det.x2) / 2;
    float cy = (det.y1 + det.y2) / 2;

    float d = GetMedianDepth(depth, det.x1, det.y1, det.x2, det.y2);
    if (d < 0) continue;

    return DeprojectToWorld(depth, cx, cy);
  }

  return {-1, -1, -1};
}

// ─── Safety / HITL fault testing ──────────────────────────────────────────────

bool Camera::IsConnected() const {
  bool disconnected = fault_injector_ && fault_injector_->IsBlocked(FaultCode::kCameraDisconnected);
  return is_open_ && !disconnected;
}

double Camera::GetFrameTimestamp() const {
  std::lock_guard<std::mutex> lock(frame_mutex_);
  return frame_timestamp_;
}

double Camera::GetDepthValue() const {
  std::lock_guard<std::mutex> lock(depth_mutex_);
  return depth_value_;
}

bool Camera::IsDetectionValid() const {
  std::lock_guard<std::mutex> lock(detections_mutex_);
  return !latest_detections_.empty();
}

// ─── Stem position ────────────────────────────────────────────────────────────

std::tuple<double, double, double> Camera::GetStemPosition() {
  if (!is_open_) {
    std::cerr << "Error: camera not opened. Call Open() first." << std::endl;
    return {-1, -1, -1};
  }

  cv::Mat frame;
  rs2::depth_frame depth{nullptr};
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    frame = latest_frame_.clone();
  }
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    depth = latest_depth_frame_;
  }

  if (frame.empty() || !depth) return {-1, -1, -1};

  auto detections = RunYolo(frame);

  for (auto& det : detections) {
    if (det.class_id != kClassStem) continue;

    float cx = (det.x1 + det.x2) / 2;
    float cy = (det.y1 + det.y2) / 2;
    int region = 10;

    float d = GetMedianDepth(depth, cx - region, cy - region, cx + region, cy + region);
    if (d < 0) continue;

    return DeprojectToWorld(depth, cx, cy);
  }

  return {-1, -1, -1};
}
