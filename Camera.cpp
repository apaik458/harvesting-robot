#include "Camera.h"

Camera::Camera() {
    // aruco setup
    aruco_dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
    aruco_params = cv::aruco::DetectorParameters::create();
    camera_matrix = (cv::Mat_<double>(3, 3) <<
        603.5139681293208, 0.0, 322.4871143178484,
        0.0, 606.2351953521359, 238.09114115221269,
        0.0, 0.0, 1.0);
    dist_coeffs = (cv::Mat_<double>(1, 5) <<
        -0.005862800130731961, 1.193475628221078,
        0.0006140022848452339, -0.0003950816468824019, -4.37080936712476);

    // load yolo onnx model
    std::string model_path = "../machine_learning/model_output/train/weights/best.onnx";
    yolo_net = cv::dnn::readNetFromONNX(model_path);
    if (yolo_net.empty()) {
        std::cerr << "Error: could not load YOLO model from " << model_path << std::endl;
    } else {
        yolo_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        yolo_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        std::cout << "YOLO model loaded." << std::endl;
    }
}

Camera::~Camera() {
    if (is_open) {
        is_open = false;
        if (capture_thread.joinable()) capture_thread.join();
        rs_pipeline.stop();
    }
}

bool Camera::open(int device_id) {
    // start realsense pipeline with colour and depth streams
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

    try {
        rs_pipeline.start(cfg);
    } catch (const rs2::error& e) {
        std::cerr << "Error: could not open RealSense camera: " << e.what() << std::endl;
        return false;
    }

    is_open = true;
    std::cout << "RealSense camera opened." << std::endl;

    // start capture thread
    capture_thread = std::thread([this]() {
        while (is_open) {
            rs2::frameset frames = rs_pipeline.wait_for_frames();

            // align depth to colour
            rs2::frameset aligned = align_to_color.process(frames);
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

            // store latest frame and depth
            {
                std::lock_guard<std::mutex> lock(frame_mutex);
                latest_frame = frame.clone();
            }
            {
                std::lock_guard<std::mutex> lock(depth_mutex);
                latest_depth_frame = depth_frame;
            }

            // draw aruco markers on preview
            std::vector<std::vector<cv::Point2f>> corners;
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> rejected;
            cv::aruco::detectMarkers(frame, aruco_dict, corners, ids, aruco_params, rejected);
            if (!ids.empty()) {
                cv::aruco::drawDetectedMarkers(frame, corners, ids);
            }

            cv::imshow("Camera Preview", frame);
            cv::waitKey(1);
        }
        cv::destroyWindow("Camera Preview");
    });

    // wait for first frame
    std::cout << "Waiting for first frame..." << std::endl;
    while (true) {
        std::lock_guard<std::mutex> lock(frame_mutex);
        if (!latest_frame.empty()) break;
    }
    std::cout << "Camera ready." << std::endl;
    return true;
}

// ─── ArUco ───────────────────────────────────────────────────────────────────

std::tuple<double, double, double> Camera::getMarkerPosition() {
    if (!is_open) {
        std::cerr << "Error: camera not opened. Call open() first." << std::endl;
        return {-1, -1, -1};
    }

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
    return result;
}

// ─── Helpers ─────────────────────────────────────────────────────────────────

float Camera::getMedianDepth(const rs2::depth_frame& depth, float x1, float y1, float x2, float y2) {
    // shrink to inner 50% of bounding box to avoid background at edges
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

std::tuple<double, double, double> Camera::deprojectToWorld(const rs2::depth_frame& depth, float cx, float cy) {
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

std::vector<Detection> Camera::runYOLO(const cv::Mat& frame) {
    std::vector<Detection> detections;

    float scale_x = (float)frame.cols / YOLO_INPUT_SIZE;
    float scale_y = (float)frame.rows / YOLO_INPUT_SIZE;

    cv::Mat blob = cv::dnn::blobFromImage(
        frame, 1.0 / 255.0,
        cv::Size(YOLO_INPUT_SIZE, YOLO_INPUT_SIZE),
        cv::Scalar(), true, false
    );

    yolo_net.setInput(blob);
    cv::Mat output = yolo_net.forward();

    // output shape: [1, 6, 8400] → reshape to [8400, 6]
    cv::Mat output_t = output.reshape(1, output.size[2]);
    cv::transpose(output_t, output_t);

    for (int i = 0; i < output_t.rows; i++) {
        float* row = output_t.ptr<float>(i);

        float cx = row[0];
        float cy = row[1];
        float w  = row[2];
        float h  = row[3];

        // find best class
        float conf_strawberry = row[4];
        float conf_stem       = row[5];

        float confidence;
        int class_id;
        if (conf_strawberry > conf_stem) {
            confidence = conf_strawberry;
            class_id = CLASS_STRAWBERRY;
        } else {
            confidence = conf_stem;
            class_id = CLASS_STEM;
        }

        if (confidence < CONF_THRESHOLD) continue;

        // scale back to frame coordinates
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

std::tuple<double, double, double> Camera::getStrawberryPosition() {
    if (!is_open) {
        std::cerr << "Error: camera not opened. Call open() first." << std::endl;
        return {-1, -1, -1};
    }

    cv::Mat frame;
    rs2::depth_frame depth{nullptr};
    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        frame = latest_frame.clone();
    }
    {
        std::lock_guard<std::mutex> lock(depth_mutex);
        depth = latest_depth_frame;
    }

    if (frame.empty() || !depth) return {-1, -1, -1};

    auto detections = runYOLO(frame);

    for (auto& det : detections) {
        if (det.class_id != CLASS_STRAWBERRY) continue;

        float cx = (det.x1 + det.x2) / 2;
        float cy = (det.y1 + det.y2) / 2;

        // use median depth over bounding box region
        float d = getMedianDepth(depth, det.x1, det.y1, det.x2, det.y2);
        if (d < 0) continue;

        return deprojectToWorld(depth, cx, cy);
    }

    return {-1, -1, -1};  // no strawberry found
}

// ─── Stem position ────────────────────────────────────────────────────────────

std::tuple<double, double, double> Camera::getStemPosition() {
    if (!is_open) {
        std::cerr << "Error: camera not opened. Call open() first." << std::endl;
        return {-1, -1, -1};
    }

    cv::Mat frame;
    rs2::depth_frame depth{nullptr};
    {
        std::lock_guard<std::mutex> lock(frame_mutex);
        frame = latest_frame.clone();
    }
    {
        std::lock_guard<std::mutex> lock(depth_mutex);
        depth = latest_depth_frame;
    }

    if (frame.empty() || !depth) return {-1, -1, -1};

    auto detections = runYOLO(frame);

    for (auto& det : detections) {
        if (det.class_id != CLASS_STEM) continue;

        float cx = (det.x1 + det.x2) / 2;
        float cy = (det.y1 + det.y2) / 2;
        int region = 10;

        // use small region median for thin stem
        float d = getMedianDepth(depth, cx - region, cy - region, cx + region, cy + region);
        if (d < 0) continue;

        return deprojectToWorld(depth, cx, cy);
    }

    return {-1, -1, -1};  // no stem found
}