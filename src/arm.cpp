#include "arm.h"

Arm::Arm(const char* port)
    : port_(port),
      port_handler_(dynamixel::PortHandler::getPortHandler(port_)),
      packet_handler_(dynamixel::PacketHandler::getPacketHandler(2.0)),
      dxl_id1_(1),
      dxl_id2_(2),
      dxl_id3_(3),
      motor1_current_position_(2048),
      motor2_current_position_(2048),
      motor3_current_position_(2048),
      max_velocity_(50),
      max_acceleration_(0),
      non_blocking_(false),
      move_accuracy_threshold_(20),
      current_x_(0.0),
      current_y_(-50.0),
      dxl_error_(0),
      dxl_comm_result_(COMM_TX_FAIL),
      dxl_addparam_result_(false),
      dxl_getdata_result_(false),
      waypoints_number_(3),
      large_movement_threshold_(20),
      fault_injector_(nullptr),
      connected_(false),
      torque_(0.0),
      motor_connected_{false, false, false},
      motor_torque_{0.0, 0.0, 0.0},
      torque_constant_Nm_per_amp(1.65)  // a reasoned approximation from datasheet for MX-106
{
}

void Arm::SetFaultInjector(FaultInjector* injector) {
  fault_injector_ = injector;
}

bool Arm::IsConnected() const {
  return connected_;
}

double Arm::GetMaxTorque() const {
  return torque_;
}

std::tuple<bool, bool, bool> Arm::GetMotorConnections() const {
  return std::make_tuple(motor_connected_[0], motor_connected_[1], motor_connected_[2]);
}

std::tuple<double, double, double> Arm::GetMotorTorques() const {
  return std::make_tuple(motor_torque_[0], motor_torque_[1], motor_torque_[2]);
}

// Establishing connection to servo motors
void Arm::Connect() {
  if (port_handler_->openPort()) {
    std::cout << "Succeeded to open the port!\n";
  } else {
    std::cout << "Failed to open the port!\n";
    return;
  }

  if (port_handler_->setBaudRate(57600)) {
    std::cout << "Succeeded to change the baudrate!\n";
  } else {
    std::cout << "Failed to change the baudrate!\n";
    return;
  }

  uint16_t torque_on_address = 64;
  uint8_t data = 1;
  dxl_comm_result_ = packet_handler_->write1ByteTxRx(port_handler_, dxl_id1_, torque_on_address, data, &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packet_handler_->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packet_handler_->getRxPacketError(dxl_error_) << std::endl;
  } else {
    std::cout << "Dynamixel#1 has been successfully connected \n";
  }

  dxl_comm_result_ = packet_handler_->write1ByteTxRx(port_handler_, dxl_id2_, torque_on_address, data, &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packet_handler_->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packet_handler_->getRxPacketError(dxl_error_) << std::endl;
  } else {
    std::cout << "Dynamixel#2 has been successfully connected \n";
  }

  dxl_comm_result_ = packet_handler_->write1ByteTxRx(port_handler_, dxl_id3_, torque_on_address, data, &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packet_handler_->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packet_handler_->getRxPacketError(dxl_error_) << std::endl;
  } else {
    std::cout << "Dynamixel#3 has been successfully connected \n";
  }

  std::tuple<int, int, int> positions = Read();
  motor1_current_position_ = std::get<0>(positions);
  motor2_current_position_ = std::get<1>(positions);
  motor3_current_position_ = std::get<2>(positions);
  connected_ = true;
}

// Gathers data from servo motors: connection statuses and torques
void Arm::PollSafetyTelemetry() {
  bool all_comm_ok = true;
  double max_torque_nm = 0.0;
  uint8_t motor_ids[3] = {dxl_id1_, dxl_id2_, dxl_id3_};

  for (int i = 0; i < 3; i++) {
    uint8_t motor_id = motor_ids[i];

    // Ensuring this motor is connected
    uint32_t raw_position;
    dxl_comm_result_ = packet_handler_->read4ByteTxRx(port_handler_, motor_id, PresentPositionAddress, &raw_position, &dxl_error_);
    bool position_comm_ok = (dxl_comm_result_ == COMM_SUCCESS) && (dxl_error_ == 0);
    all_comm_ok = all_comm_ok && position_comm_ok;
    motor_connected_[i] = position_comm_ok;

    // Reading this motor's torque
    uint16_t raw_current;
    dxl_comm_result_ = packet_handler_->read2ByteTxRx(port_handler_, motor_id, PresentCurrentAddress, &raw_current, &dxl_error_);
    if (dxl_comm_result_ != COMM_SUCCESS || dxl_error_ != 0) continue;

    int16_t signed_current_ma = static_cast<int16_t>(raw_current);
    double current_amps = std::abs(signed_current_ma) / 1000.0;
    if (fault_injector_) {
      current_amps = fault_injector_->ApplyToValue(current_amps, FaultCode::ExcessiveTorque);
    }
    double torque_nm = current_amps * torque_constant_Nm_per_amp;
    motor_torque_[i] = torque_nm;
    max_torque_nm = std::max(max_torque_nm, torque_nm);
  }

  bool disconnected = fault_injector_ && fault_injector_->IsBlocked(FaultCode::ServoDisconnected);
  connected_ = all_comm_ok && !disconnected;
  torque_ = max_torque_nm;
}

std::tuple<int, int, int> Arm::Read() {
  uint32_t present_position1;
  uint32_t present_position2;
  uint32_t present_position3;
  dxl_comm_result_ = packet_handler_->read4ByteTxRx(port_handler_, dxl_id1_, PresentPositionAddress, &present_position1, &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packet_handler_->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packet_handler_->getRxPacketError(dxl_error_) << std::endl;
  }
  dxl_comm_result_ = packet_handler_->read4ByteTxRx(port_handler_, dxl_id2_, PresentPositionAddress, &present_position2, &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packet_handler_->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packet_handler_->getRxPacketError(dxl_error_) << std::endl;
  }
  dxl_comm_result_ = packet_handler_->read4ByteTxRx(port_handler_, dxl_id3_, PresentPositionAddress, &present_position3, &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packet_handler_->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packet_handler_->getRxPacketError(dxl_error_) << std::endl;
  }
  motor1_current_position_ = present_position1;
  motor2_current_position_ = present_position2;
  motor3_current_position_ = present_position3;

  return std::make_tuple(present_position1, present_position2, present_position3);
}

void Arm::Write(int target_position1, int target_position2, int target_position3) {
  // Determine what speed is required for each motor to reach the target position at the same time
  std::tuple<double, double, double> speeds = CalculateSpeeds(target_position1, target_position2, target_position3);
  double speed1 = std::get<0>(speeds);
  double speed2 = std::get<1>(speeds);
  double speed3 = std::get<2>(speeds);

  std::cout << "speeds: " << speed1 << ", " << speed2 << ", " << speed3 << std::endl;

  packet_handler_->write4ByteTxRx(port_handler_, dxl_id1_, ProfileVelocityAddress, speed1, &dxl_error_);
  packet_handler_->write4ByteTxRx(port_handler_, dxl_id2_, ProfileVelocityAddress, speed2, &dxl_error_);
  packet_handler_->write4ByteTxRx(port_handler_, dxl_id3_, ProfileVelocityAddress, speed3, &dxl_error_);

  // to control motor acceleration (for trapezoidal velocity profile)
  packet_handler_->write4ByteTxRx(port_handler_, dxl_id1_, ProfileAccelerationAddress, max_acceleration_, &dxl_error_);
  packet_handler_->write4ByteTxRx(port_handler_, dxl_id2_, ProfileAccelerationAddress, max_acceleration_, &dxl_error_);
  packet_handler_->write4ByteTxRx(port_handler_, dxl_id3_, ProfileAccelerationAddress, max_acceleration_, &dxl_error_);

  dxl_comm_result_ = packet_handler_->write4ByteTxRx(port_handler_, dxl_id1_, GoalPositionAddress, uint32_t(target_position1), &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packet_handler_->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packet_handler_->getRxPacketError(dxl_error_) << std::endl;
  }

  dxl_comm_result_ = packet_handler_->write4ByteTxRx(port_handler_, dxl_id2_, GoalPositionAddress, uint32_t(target_position2), &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packet_handler_->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packet_handler_->getRxPacketError(dxl_error_) << std::endl;
  }

  dxl_comm_result_ = packet_handler_->write4ByteTxRx(port_handler_, dxl_id3_, GoalPositionAddress, uint32_t(target_position3), &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packet_handler_->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packet_handler_->getRxPacketError(dxl_error_) << std::endl;
  }

  if (!non_blocking_) {
    int present_position1, present_position2, present_position3;
    do {
      std::tuple<int, int, int> positions = Read();
      present_position1 = std::get<0>(positions);
      present_position2 = std::get<1>(positions);
      present_position3 = std::get<2>(positions);
      std::cout << "Current Position: " << present_position1 << ", " << present_position2 << ", " << present_position3 << std::endl;
    } while (abs(target_position1 - present_position1) > move_accuracy_threshold_ || abs(target_position2 - present_position2) > move_accuracy_threshold_ || abs(target_position3 - present_position3) > move_accuracy_threshold_);
  }
}

// Blocking arm movement used only for large movements so arm can move in a straight line without overextending
void Arm::WriteWaypoints(int target_position1, int target_position2, int target_position3, double target_x, double target_y) {
  int present_position1, present_position2, present_position3;
  std::tuple<int, int, int> positions = Read();
  present_position1 = std::get<0>(positions);
  present_position2 = std::get<1>(positions);
  present_position3 = std::get<2>(positions);

  // 1. Get current cartesian position via FK
  auto [curr_x, curr_y] = CalculateForwardKinematics(present_position1, present_position2, present_position3);
  std::cout << "FK current position: x=" << curr_x << ", y=" << curr_y << std::endl;

  // 2. Generate equally spaced waypoints in cartesian space, convert each to joint space via IK
  std::vector<int> waypoints1, waypoints2, waypoints3;
  for (int i = 1; i <= waypoints_number_; i++) {
    double t = (double)i / waypoints_number_;
    double wp_x = curr_x + t * (target_x - curr_x);
    double wp_y = curr_y + t * (target_y - curr_y);

    auto [m1, m2, m3] = CalculateInverseKinematics(wp_x, wp_y);
    if (m1 == -1) {
      std::cout << "IK out of bounds at waypoint " << i << ", skipping" << std::endl;
      continue;
    }
    std::cout << "Waypoint " << i << ": x=" << wp_x << ", y=" << wp_y << " -> motors: " << m1 << ", " << m2 << ", " << m3 << std::endl;
    waypoints1.push_back(m1);
    waypoints2.push_back(m2);
    waypoints3.push_back(m3);
  }

  // 3. Set speeds once (based on final target)
  std::tuple<double, double, double> speeds = CalculateSpeeds(target_position1, target_position2, target_position3);
  double speed1 = std::get<0>(speeds);
  double speed2 = std::get<1>(speeds);
  double speed3 = std::get<2>(speeds);
  std::cout << "speeds: " << speed1 << ", " << speed2 << ", " << speed3 << std::endl;
  packet_handler_->write4ByteTxRx(port_handler_, dxl_id1_, ProfileVelocityAddress, speed1, &dxl_error_);
  packet_handler_->write4ByteTxRx(port_handler_, dxl_id2_, ProfileVelocityAddress, speed2, &dxl_error_);
  packet_handler_->write4ByteTxRx(port_handler_, dxl_id3_, ProfileVelocityAddress, speed3, &dxl_error_);

  // 4. Iterate through each waypoint
  int total_waypoints = waypoints1.size();
  for (int idx = 0; idx < total_waypoints; idx++) {
    // bool is_first = (idx == 0);
    // bool is_last  = (idx == total_waypoints - 1);
    // uint32_t accel = (is_first || is_last) ? max_acceleration_ : 0;

    uint32_t accel = max_acceleration_;
    packet_handler_->write4ByteTxRx(port_handler_, dxl_id1_, ProfileAccelerationAddress, accel, &dxl_error_);
    packet_handler_->write4ByteTxRx(port_handler_, dxl_id2_, ProfileAccelerationAddress, accel, &dxl_error_);
    packet_handler_->write4ByteTxRx(port_handler_, dxl_id3_, ProfileAccelerationAddress, accel, &dxl_error_);

    dxl_comm_result_ = packet_handler_->write4ByteTxRx(port_handler_, dxl_id1_, GoalPositionAddress, uint32_t(waypoints1[idx]), &dxl_error_);
    if (dxl_comm_result_ != COMM_SUCCESS) {
      std::cout << packet_handler_->getTxRxResult(dxl_comm_result_) << std::endl;
    } else if (dxl_error_ != 0) {
      std::cout << packet_handler_->getRxPacketError(dxl_error_) << std::endl;
    }
    dxl_comm_result_ = packet_handler_->write4ByteTxRx(port_handler_, dxl_id2_, GoalPositionAddress, uint32_t(waypoints2[idx]), &dxl_error_);
    if (dxl_comm_result_ != COMM_SUCCESS) {
      std::cout << packet_handler_->getTxRxResult(dxl_comm_result_) << std::endl;
    } else if (dxl_error_ != 0) {
      std::cout << packet_handler_->getRxPacketError(dxl_error_) << std::endl;
    }
    dxl_comm_result_ = packet_handler_->write4ByteTxRx(port_handler_, dxl_id3_, GoalPositionAddress, uint32_t(waypoints3[idx]), &dxl_error_);
    if (dxl_comm_result_ != COMM_SUCCESS) {
      std::cout << packet_handler_->getTxRxResult(dxl_comm_result_) << std::endl;
    } else if (dxl_error_ != 0) {
      std::cout << packet_handler_->getRxPacketError(dxl_error_) << std::endl;
    }

    do {
      std::tuple<int, int, int> positions = Read();
      present_position1 = std::get<0>(positions);
      present_position2 = std::get<1>(positions);
      present_position3 = std::get<2>(positions);
      std::cout << "Current Position: " << present_position1 << ", " << present_position2 << ", " << present_position3 << std::endl;
    } while (abs(waypoints1[idx] - present_position1) > move_accuracy_threshold_ * 5 ||
             abs(waypoints2[idx] - present_position2) > move_accuracy_threshold_ * 5 ||
             abs(waypoints3[idx] - present_position3) > move_accuracy_threshold_ * 5);
  }
}

void Arm::Write(std::string command, double x, double y) {
  if (command == "cartesian") {
    std::tuple<int, int, int> joint_commands = CalculateInverseKinematics(x, y);
    if (std::get<0>(joint_commands) == -1) {
      std::cout << "Target position is out of bounds for the arm." << std::endl;
      return;
    }
    // Get current motor positions and convert to cartesian
    std::tuple<int, int, int> current_positions = Read();
    auto [curr_x, curr_y] = CalculateForwardKinematics(std::get<0>(current_positions), std::get<1>(current_positions), std::get<2>(current_positions));

    // Calculate cartesian distance to target
    double dx = x - curr_x;
    double dy = y - curr_y;
    double movement_distance = sqrt(dx * dx + dy * dy);
    std::cout << "Movement distance: " << movement_distance << "cm" << std::endl;

    bool movement_distance_large = movement_distance > large_movement_threshold_;

    if (movement_distance_large) {
      WriteWaypoints(std::get<0>(joint_commands), std::get<1>(joint_commands), std::get<2>(joint_commands), x, y);
    } else {
      Write(std::get<0>(joint_commands), std::get<1>(joint_commands), std::get<2>(joint_commands));
    }
  } else {
    std::cout << "Unknown command: " << command << std::endl;
  }
  return;
}

void Arm::Write(std::string command, double x, double y, int velocity) {
  max_velocity_ = velocity;
  max_acceleration_ = 0;
  Write(command, x, y);
}

void Arm::Write(std::string command, double x, double y, int velocity, int acceleration) {
  max_velocity_ = velocity;
  max_acceleration_ = acceleration;
  Write(command, x, y);
}

void Arm::NonBlockingState(bool block) {
  non_blocking_ = block;
}




// Helper functions
//
//

std::tuple<double, double, double> Arm::CalculateSpeeds(int target_position1, int target_position2, int target_position3) {
  std::tuple<int, int, int> positions = Read();
  int present_position1 = std::get<0>(positions);
  int present_position2 = std::get<1>(positions);
  int present_position3 = std::get<2>(positions);

  double dists[3] = {
    fabs((target_position1 - present_position1) / 4096.0),  // in units of revs
    fabs((target_position2 - present_position2) / 4096.0),
    fabs((target_position3 - present_position3) / 4096.0)
  };

  if (max_acceleration_ == 0) {
    int dist1 = abs(target_position1 - present_position1);
    int dist2 = abs(target_position2 - present_position2);
    int dist3 = abs(target_position3 - present_position3);

    // 1. Find the motor with the largest distance
    int max_dist = std::max({dist1, dist2, dist3});
    if (max_dist == 0) return std::make_tuple(0, 0, 0);

    // 2. Back calculate speeds for other motors so they finish at the same time
    // speed = distance / time = distance / (max_dist / max_velocity)
    //       = (distance * max_velocity) / max_dist
    double speed1 = (dist1 * max_velocity_) / (double)max_dist;
    double speed2 = (dist2 * max_velocity_) / (double)max_dist;
    double speed3 = (dist3 * max_velocity_) / (double)max_dist;
    return std::make_tuple(speed1, speed2, speed3);
  } else {
    // 1. Find out the distance threshold where velocity profile goes from triangular to trapezoidal
    double vel_rev_min = max_velocity_ * 0.229;
    double acc_rev_min_squared = max_acceleration_ * 214.577;  // converting to consistent units, from Dynamixel XM430 Control Table

    double acc_constant = tan((M_PI / 2) - atan(acc_rev_min_squared));
    double t_threshold = 2 * (vel_rev_min * acc_constant);
    double dist_threshold = (t_threshold * vel_rev_min) / 2.0;

    // 2. Find out the time that the farthest away servo takes to reach target
    double max_dist = *std::max_element(std::begin(dists), std::end(dists));
    if (max_dist == 0) return std::make_tuple(0, 0, 0);

    double t;
    if (max_dist > dist_threshold) {  // trapezoid
      t = (2 * (max_dist / vel_rev_min) + t_threshold) / 2;
    } else {  // triangle
      t = sqrt((4 * max_dist) / acc_rev_min_squared);
    }

    // 3. Knowing the time and distance of each movement, and which velocity profile to use, max velocities can be found
    double vels[3];
    double vel;
    for (int i = 0; i < 3; i++) {
      double dist = dists[i];
      if (dist > dist_threshold) {  // trapezoid
        vel = (-t - sqrt((t * t) - (4 * -acc_constant * -dist))) / (-2 * acc_constant);
      } else {  // triangle
        vel = (2 * dist) / t;
      }
      vel = vel / 0.229;  // conversion back into a usable unit
      vels[i] = vel;
    }

    return std::make_tuple(vels[0], vels[1], vels[2]);
  }
}

std::tuple<int, int, int> Arm::CalculateInverseKinematics(double x, double y) {
  double distance_from_origin = sqrt(x * x + y * y);

  // Elbow angle (motor 2): apply cosine rule
  double elbow_angle = M_PI - acos((Link1LengthCm * Link1LengthCm + Link2LengthCm * Link2LengthCm - distance_from_origin * distance_from_origin) / (2 * Link1LengthCm * Link2LengthCm));
  if (x < 0) {
    elbow_angle = -elbow_angle;  // Ensures CW elbow rotation if target is in the left half plane
  }

  // Shoulder angle (motor 1):
  // Step 1: first finding elbow joint coordinate - this logic required a long hand calculation
  double K = -Link1LengthCm * Link1LengthCm - x * x - y * y + Link2LengthCm * Link2LengthCm;

  std::cout << "K value: " << K << std::endl;

  double a = 4 * x * x + 4 * y * y;
  double b = 4 * y * K;
  double c = K * K - 4 * x * x * Link1LengthCm * Link1LengthCm;

  std::cout << "Quadratic coefficients: a=" << a << ", b=" << b << ", c=" << c << std::endl;

  double discriminant = b * b - 4 * a * c;
  std::cout << "Discriminant: " << discriminant << std::endl;
  double elbow_y1 = (-b + sqrt(discriminant)) / (2 * a);
  double elbow_y2 = (-b - sqrt(discriminant)) / (2 * a);

  double elbow_y = std::min(elbow_y1, elbow_y2);

  std::cout << "Elbow Y candidates: " << elbow_y1 << ", " << elbow_y2 << std::endl;

  double elbow_x_pos = sqrt(Link1LengthCm * Link1LengthCm - elbow_y * elbow_y);
  double elbow_x_neg = -elbow_x_pos;

  std::cout << "Elbow X candidates: " << elbow_x_pos << ", " << elbow_x_neg << std::endl;

  double err_pos = pow(elbow_x_pos - x, 2) + pow(elbow_y - y, 2) - Link2LengthCm * Link2LengthCm;
  double err_neg = pow(elbow_x_neg - x, 2) + pow(elbow_y - y, 2) - Link2LengthCm * Link2LengthCm;

  std::cout << "Error for positive elbow x: " << err_pos << ", Error for negative elbow x: " << err_neg << std::endl;

  double elbow_x = (abs(err_pos) < abs(err_neg)) ? elbow_x_pos : elbow_x_neg;

  std::cout << "Chosen elbow coordinate: (" << elbow_x << ", " << elbow_y << ")" << std::endl;

  // Step 2: shoulder angle can now be found using trig with the elbow joint coordinate as reference:
  double shoulder_angle = atan2(elbow_y, elbow_x) + M_PI / 2;
  if (shoulder_angle > M_PI) shoulder_angle -= 2 * M_PI;  // wraparound logic
  if (shoulder_angle < -M_PI) shoulder_angle += 2 * M_PI;

  std::cout << "Calculated angles (radians): Shoulder: " << shoulder_angle << ", Elbow: " << elbow_angle << std::endl;

  double wrist_angle = (x > 0) ? -1 * ((M_PI / 2) - (shoulder_angle + elbow_angle)) : -1 * ((-M_PI / 2) - (shoulder_angle + elbow_angle));
  if (x == 0) {
    wrist_angle = 0;  // sits straight when at rest
  }

  // Convert the angles to motor positions (0-4095)
  int motor1_position = 2048 + int((shoulder_angle / (2.0 * M_PI)) * 4096);
  int motor2_position = 2048 + int((elbow_angle / (2.0 * M_PI)) * 4096);
  int motor3_position = 2048 + int((wrist_angle / (2.0 * M_PI)) * 4096);

  std::cout << "Calculated motor positions: " << motor1_position << ", " << motor2_position << ", " << motor3_position << std::endl;

  if (motor1_position < 342 || motor1_position > 3754 ||
      motor2_position < 342 || motor2_position > 3754 ||
      motor3_position < 342 || motor3_position > 3754) {
    return std::make_tuple(-1, -1, -1);  // Return an error code if the target position is out of bounds
  }

  return std::make_tuple(motor1_position, motor2_position, motor3_position);
}

std::tuple<double, double> Arm::CalculateForwardKinematics(int motor1_position, int motor2_position, int motor3_position) {
  // Convert motor positions back to angles
  double shoulder_angle = ((motor1_position - 2048) / 4096.0) * (2.0 * M_PI);
  double elbow_angle    = ((motor2_position - 2048) / 4096.0) * (2.0 * M_PI);

  // Shoulder angle was stored as atan2(elbow_y, elbow_x) + pi/2
  // so the actual link1 angle from vertical is shoulder_angle - pi/2
  double link1_angle = shoulder_angle - M_PI / 2.0;

  // Elbow joint position (end of link1)
  double elbow_x = Link1LengthCm * cos(link1_angle);
  double elbow_y = Link1LengthCm * sin(link1_angle);

  // Elbow angle was stored as pi - acos(...), and negated for x<0
  // link2 direction = link1_angle + elbow_angle (chained joints)
  double link2_angle = link1_angle + elbow_angle;

  // End effector position (end of link2)
  double x = elbow_x + Link2LengthCm * cos(link2_angle);
  double y = elbow_y + Link2LengthCm * sin(link2_angle);

  std::cout << "FK result: x=" << x << ", y=" << y << std::endl;

  return std::make_tuple(x, y);
}
