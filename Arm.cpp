#include "Arm.h"

Arm::Arm(const char * port) 
    : port_(port), 
    portHandler(dynamixel::PortHandler::getPortHandler(port_)), 
    packetHandler(dynamixel::PacketHandler::getPacketHandler(2.0)),
    dxl_id1_(1),
    dxl_id2_(2),
    dxl_id3_(3),
    motor1_current_position_(2048),
    motor2_current_position_(2048),
    motor3_current_position_(2048),
    max_velocity_(50),
    max_acceleration_(3),
    non_blocking_(false),
    move_accuracy_threshold_(20),
    current_x_(0.0),
    current_y_(-50.0),
    dxl_error_(0),
    dxl_comm_result_(COMM_TX_FAIL),
    dxl_addparam_result_(false),
    dxl_getdata_result_(false),
    target_x_(0.0),
    target_y_(-50.0)
{
}

void Arm::connect() {
  if (portHandler->openPort()) {
    std::cout << "Succeeded to open the port!\n";
  } else {
    std::cout << "Failed to open the port!\n";
    return;
  }

  if (portHandler->setBaudRate(57600)) {
    std::cout << "Succeeded to change the baudrate!\n";
  } else {
    std::cout << "Failed to change the baudrate!\n";
    return;
  }

  uint16_t torque_on_address = 64;
  uint8_t data = 1;
  dxl_comm_result_ = packetHandler->write1ByteTxRx(portHandler, dxl_id1_, torque_on_address, data, &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packetHandler->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packetHandler->getRxPacketError(dxl_error_) << std::endl;
  } else {
    std::cout << "Dynamixel#1 has been successfully connected \n";
  }

  dxl_comm_result_ = packetHandler->write1ByteTxRx(portHandler, dxl_id2_, torque_on_address, data, &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packetHandler->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packetHandler->getRxPacketError(dxl_error_) << std::endl;
  } else {
    std::cout << "Dynamixel#2 has been successfully connected \n";
  }

  dxl_comm_result_ = packetHandler->write1ByteTxRx(portHandler, dxl_id3_, torque_on_address, data, &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packetHandler->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packetHandler->getRxPacketError(dxl_error_) << std::endl;
  } else {
    std::cout << "Dynamixel#3 has been successfully connected \n";
  }

  std::tuple<int, int, int> positions = read();
  motor1_current_position_ = std::get<0>(positions);
  motor2_current_position_ = std::get<1>(positions);
  motor3_current_position_ = std::get<2>(positions);
}

std::tuple<int, int, int> Arm::read() {
  uint32_t present_position1;
  uint32_t present_position2;
  uint32_t present_position3;
  dxl_comm_result_ = packetHandler->read4ByteTxRx(portHandler, dxl_id1_, present_position_address, &present_position1, &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packetHandler->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packetHandler->getRxPacketError(dxl_error_) << std::endl;
  }
  dxl_comm_result_ = packetHandler->read4ByteTxRx(portHandler, dxl_id2_, present_position_address, &present_position2, &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packetHandler->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packetHandler->getRxPacketError(dxl_error_) << std::endl;
  }
  dxl_comm_result_ = packetHandler->read4ByteTxRx(portHandler, dxl_id3_, present_position_address, &present_position3, &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packetHandler->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packetHandler->getRxPacketError(dxl_error_) << std::endl;
  }
  motor1_current_position_ = present_position1;
  motor2_current_position_ = present_position2;
  motor3_current_position_ = present_position3;

  return std::make_tuple(present_position1, present_position2, present_position3);
}

void Arm::write(int target_position1, int target_position2, int target_position3) {
  // Determine what speed is required for each motor to reach the target position at the same time
  std::tuple<double, double, double> speeds = calculateSpeeds(target_position1, target_position2, target_position3);
  double speed1 = std::get<0>(speeds);
  double speed2 = std::get<1>(speeds);
  double speed3 = std::get<2>(speeds);

  std::cout << "speeds: " << speed1 << ", " << speed2 << ", " << speed3 << std::endl;

  packetHandler->write4ByteTxRx(portHandler, dxl_id1_, profile_velocity_address, speed1, &dxl_error_);
  packetHandler->write4ByteTxRx(portHandler, dxl_id2_, profile_velocity_address, speed2, &dxl_error_);
  packetHandler->write4ByteTxRx(portHandler, dxl_id3_, profile_velocity_address, speed3, &dxl_error_);

  // to control motor acceleration (for trapezoidal velocity profile)
  packetHandler->write4ByteTxRx(portHandler, dxl_id1_, profile_acceleration_address, max_acceleration_, &dxl_error_);
  packetHandler->write4ByteTxRx(portHandler, dxl_id2_, profile_acceleration_address, max_acceleration_, &dxl_error_);
  packetHandler->write4ByteTxRx(portHandler, dxl_id3_, profile_acceleration_address, max_acceleration_, &dxl_error_);

  dxl_comm_result_ = packetHandler->write4ByteTxRx(portHandler, dxl_id1_, goal_position_address, uint32_t(target_position1), &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packetHandler->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packetHandler->getRxPacketError(dxl_error_) << std::endl;
  }

  dxl_comm_result_ = packetHandler->write4ByteTxRx(portHandler, dxl_id2_, goal_position_address, uint32_t(target_position2), &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packetHandler->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packetHandler->getRxPacketError(dxl_error_) << std::endl;
  }

  dxl_comm_result_ = packetHandler->write4ByteTxRx(portHandler, dxl_id3_, goal_position_address, uint32_t(target_position3), &dxl_error_);
  if (dxl_comm_result_ != COMM_SUCCESS) {
    std::cout << packetHandler->getTxRxResult(dxl_comm_result_) << std::endl;
  } else if (dxl_error_ != 0) {
    std::cout << packetHandler->getRxPacketError(dxl_error_) << std::endl;
  }

  if (!non_blocking_) {
    int present_position1, present_position2, present_position3;
    do {
      std::tuple<int, int, int> positions = read();
      present_position1 = std::get<0>(positions);
      present_position2 = std::get<1>(positions);
      present_position3 = std::get<2>(positions);
      std::cout << "Current Position: " << present_position1 << ", " << present_position2 << ", " << present_position3 << std::endl;
    } while (abs(target_position1 - present_position1) > move_accuracy_threshold_ || abs(target_position2 - present_position2) > move_accuracy_threshold_ || abs(target_position3 - present_position3) > move_accuracy_threshold_);
  }
}

void Arm::write(std::string command, double x, double y) {
  if (command == "cartesian") {
    int steps = 2;
    for (int i=1;i<=steps;i++) {
      if (i == 1) {
        target_x_ = x;
        target_y_ = y;
      }
      double step_target_x = target_x_ - ((target_x_-current_x_)/steps)*(steps-i);
      double step_target_y = target_y_ - ((target_y_-current_y_)/steps)*(steps-i);
      std::tuple<int, int, int> joint_commands = calculateInverseKinematics(step_target_x, step_target_y);
      if (std::get<0>(joint_commands) == -1) {
        std::cout << "Target position is out of bounds for the arm." << std::endl;
        return;
      }
      // if (!non_blocking_) {
      //   std::tuple<int, int, int> positions = read();
      //   int present_position1 = std::get<0>(positions);
      //   int present_position2 = std::get<1>(positions);
      //   int present_position3 = std::get<2>(positions);
      //   int steps = 4;
      //   for (int i=1;i<=steps;i++) {
      //     write(present_position1 - (((present_position1-std::get<0>(joint_commands))/steps)*i), present_position2 - (((present_position2-std::get<1>(joint_commands))/steps)*i), present_position3 - (((present_position3-std::get<2>(joint_commands))/steps)*i));
      //   }
      // } else {
      //   write(std::get<0>(joint_commands), std::get<1>(joint_commands), std::get<2>(joint_commands)); 
      // }
      write(std::get<0>(joint_commands), std::get<1>(joint_commands), std::get<2>(joint_commands));
      if (i == steps) {
        current_x_ = target_x_;
        current_y_ = target_y_;
      }
    } 
  } else {
    std::cout << "Unknown command: " << command << std::endl;
  }
  return;
}

void Arm::write(std::string command, double x, double y, int velocity) {
  max_velocity_ = velocity;
  write(command, x, y);
}

void Arm::write(std::string command, double x, double y, int velocity, int acceleration) {
  max_velocity_ = velocity;
  max_acceleration_ = acceleration;
  write(command, x, y);
}

void Arm::move_non_blocking() {
  non_blocking_ = true;
}




// Helper functions
//
//

std::tuple<double, double, double> Arm::calculateSpeeds(int target_position1, int target_position2, int target_position3) {
    std::tuple<int, int, int> positions = read();
    int present_position1 = std::get<0>(positions);
    int present_position2 = std::get<1>(positions);
    int present_position3 = std::get<2>(positions);

    double dists[3] = {
      fabs((target_position1 - present_position1) / 4096.0),  // in units of revs
      fabs((target_position2 - present_position2) / 4096.0),
      fabs((target_position3 - present_position3) / 4096.0)
    };

    // 1. Find out the distance threshold where velocity profile goes from triangular to trapezoidal
    double vel_rev_min = max_velocity_ * 0.229; 
    double acc_rev_min_squared = max_acceleration_ * 214.577; // converting to consistent units, from Dynamixel XM430 Control Table
    
    double acc_constant = tan((M_PI/2)-atan(acc_rev_min_squared));
    double t_threshold = 2*(vel_rev_min*acc_constant);
    double dist_threshold = (t_threshold * vel_rev_min)/2.0;

    // 2. Find out the time that the farthest away servo takes to reach target
    double max_dist = *std::max_element(std::begin(dists), std::end(dists));
    if (max_dist == 0) return std::make_tuple(0, 0, 0);

    double t;
    if (max_dist > dist_threshold) { // trapezoid
      t = (2*(max_dist/vel_rev_min)+t_threshold)/2;
    } else { // triangle
      t = sqrt((4*max_dist)/acc_rev_min_squared);
    }

    //3. Knowing the time and distance of each movement, and which velocity profile to use, max velocities can be found
    double vels[3];
    double vel;
    for (int i=0; i<3; i++) {
      double dist = dists[i];
      if (dist > dist_threshold) { // trapezoid
        vel = (-t-sqrt((t*t)-(4*-acc_constant*-dist)))/(-2*acc_constant);
      } else { // triangle
        vel = (2*dist)/t;
      }
      vel = vel / 0.229; // conversion back into a usable unit
      vels[i] = vel;
    }

    return std::make_tuple(vels[0], vels[1], vels[2]);
}

std::tuple<int, int, int> Arm::calculateInverseKinematics(double x, double y) {
    double distance_from_origin = sqrt(x*x + y*y);

    // Elbow angle (motor 2): apply cosine rule
    double elbow_angle = M_PI - acos((link1_length_cm*link1_length_cm + link2_length_cm*link2_length_cm - distance_from_origin*distance_from_origin) / (2 * link1_length_cm * link2_length_cm));
    if (x < 0) {
        elbow_angle = -elbow_angle; // Ensures CW elbow rotation if target is in the left half plane
    }

    // Shoulder angle (motor 1):
    // Step 1: first finding elbow joint coordinate - this logic required a long hand calculation
    double K = -link1_length_cm*link1_length_cm - x*x - y*y + link2_length_cm*link2_length_cm;

    std::cout << "K value: " << K << std::endl;

    double a = 4*x*x+4*y*y;
    double b = 4*y*K;
    double c = K*K - 4*x*x*link1_length_cm*link1_length_cm;

    std::cout << "Quadratic coefficients: a=" << a << ", b=" << b << ", c=" << c << std::endl;

    double discriminant = b*b - 4*a*c;
    std::cout << "Discriminant: " << discriminant << std::endl;
    double elbow_y1 = (-b + sqrt(discriminant)) / (2*a);
    double elbow_y2 = (-b - sqrt(discriminant)) / (2*a);

    double elbow_y = std::min(elbow_y1, elbow_y2);

    std::cout << "Elbow Y candidates: " << elbow_y1 << ", " << elbow_y2 << std::endl;

    double elbow_x_pos = sqrt(link1_length_cm*link1_length_cm - elbow_y*elbow_y);
    double elbow_x_neg = -elbow_x_pos;

    std::cout << "Elbow X candidates: " << elbow_x_pos << ", " << elbow_x_neg << std::endl;

    double err_pos = pow(elbow_x_pos - x, 2) + pow(elbow_y - y, 2) - link2_length_cm*link2_length_cm;
    double err_neg = pow(elbow_x_neg - x, 2) + pow(elbow_y - y, 2) - link2_length_cm*link2_length_cm;

    std::cout << "Error for positive elbow x: " << err_pos << ", Error for negative elbow x: " << err_neg << std::endl;

    double elbow_x = (abs(err_pos) < abs(err_neg)) ? elbow_x_pos : elbow_x_neg;

    std::cout << "Chosen elbow coordinate: (" << elbow_x << ", " << elbow_y << ")" << std::endl;

    // Step 2: shoulder angle can now be found using trig with the elbow joint coordinate as reference:
    double shoulder_angle = atan2(elbow_y, elbow_x) + M_PI/2;
    if (shoulder_angle > M_PI) shoulder_angle -= 2*M_PI; // wraparound logic
    if (shoulder_angle < -M_PI) shoulder_angle += 2*M_PI;

    std::cout << "Calculated angles (radians): Shoulder: " << shoulder_angle << ", Elbow: " << elbow_angle << std::endl;

    double wrist_angle = (x > 0) ? -1*((M_PI/2) - (shoulder_angle + elbow_angle)) : -1*((-M_PI/2) - (shoulder_angle + elbow_angle));
    if (x == 0) {
      wrist_angle = 0; // sits straight when at rest
    }

    // Convert the angles to motor positions (0-4095)
    int motor1_position = 2048 + int((shoulder_angle / (2.0*M_PI)) * 4096);
    int motor2_position = 2048 + int((elbow_angle / (2.0*M_PI)) * 4096);
    int motor3_position = 2048 + int((wrist_angle / (2.0*M_PI)) * 4096);

    std::cout << "Calculated motor positions: " << motor1_position << ", " << motor2_position << ", " << motor3_position << std::endl;

    if (motor1_position < 512 || motor1_position > 3583 ||
        motor2_position < 512 || motor2_position > 3583 ||
        motor3_position < 512 || motor3_position > 3583) {
          return std::make_tuple(-1, -1, -1); // Return an error code if the target position is out of bounds
    }

    return std::make_tuple(motor1_position, motor2_position, motor3_position);
}