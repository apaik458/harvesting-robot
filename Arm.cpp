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
    current_x_(0.0),
    current_y_(-50.0),
    dxl_error_(0),
    dxl_comm_result_(COMM_TX_FAIL),
    dxl_addparam_result_(false),
    dxl_getdata_result_(false)
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
  return std::make_tuple(present_position1, present_position2, present_position3);
}

void Arm::write(int target_position1, int target_position2, int target_position3) {
  // Determine what speed is required for each motor to reach the target position at the same time
  std::tuple<int, int, int> speeds = calculateSpeeds(target_position1, target_position2, target_position3);
  int speed1 = std::get<0>(speeds);
  int speed2 = std::get<1>(speeds);
  int speed3 = std::get<2>(speeds);
  packetHandler->write4ByteTxRx(portHandler, dxl_id1_, profile_velocity_address, speed1, &dxl_error_);
  packetHandler->write4ByteTxRx(portHandler, dxl_id2_, profile_velocity_address, speed2, &dxl_error_);
  packetHandler->write4ByteTxRx(portHandler, dxl_id3_, profile_velocity_address, speed3, &dxl_error_);

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

  int present_position1, present_position2, present_position3;
  do {
    std::tuple<int, int, int> positions = read();
    present_position1 = std::get<0>(positions);
    present_position2 = std::get<1>(positions);
    present_position3 = std::get<2>(positions);
    std::cout << "Current Position: " << present_position1 << ", " << present_position2 << ", " << present_position3 << std::endl;
  } while (abs(target_position1 - present_position1) > 10 || abs(target_position2 - present_position2) > 10 || abs(target_position3 - present_position3) > 10);
}

void Arm::write(std::string command, float x, float y) {
  if (command == "cartesian") {
    std::tuple<int, int, int> joint_commands = calculateInverseKinematics(x, y);
    std::cout << "Calculated joint positions: " << std::get<0>(joint_commands) << ", " << std::get<1>(joint_commands) << ", " << std::get<2>(joint_commands) << std::endl;
    // write(std::get<0>(joint_commands), std::get<1>(joint_commands), std::get<2>(joint_commands));
  } else {
    std::cout << "Unknown command: " << command << std::endl;
  }
  return;
}




// Helper functions
//
//

std::tuple<int, int, int> Arm::calculateSpeeds(int target_position1, int target_position2, int target_position3) {
    int max_velocity = 50;

    std::tuple<int, int, int> positions = read();
    int present_position1 = std::get<0>(positions);
    int present_position2 = std::get<1>(positions);
    int present_position3 = std::get<2>(positions);

    int dist1 = abs(target_position1 - present_position1);
    int dist2 = abs(target_position2 - present_position2);
    int dist3 = abs(target_position3 - present_position3);

    // 1. Find the motor with the largest distance
    int max_dist = std::max({dist1, dist2, dist3});
    if (max_dist == 0) return std::make_tuple(0, 0, 0);

    // 2. Back calculate speeds for other motors so they finish at the same time
    // speed = distance / time = distance / (max_dist / max_velocity)
    //       = (distance * max_velocity) / max_dist
    int speed1 = (dist1 * max_velocity) / max_dist;
    int speed2 = (dist2 * max_velocity) / max_dist;
    int speed3 = (dist3 * max_velocity) / max_dist;

    return std::make_tuple(speed1, speed2, speed3);
}

std::tuple<int, int, int> Arm::calculateInverseKinematics(float x, float y) {
    // 1. Figure out the joint angles required to reach the target (x, y) position
    float distance_from_origin = sqrt(x*x + y*y);

    // Cosine rule. Answers are in radians, relative to arm origin position:
    float elbow_angle = M_PI - (acos((link1_length_cm*link1_length_cm + link2_length_cm*link2_length_cm - distance_from_origin*distance_from_origin) / (2*link1_length_cm*link2_length_cm)));
    float shoulder_angle = -1*((acos((link1_length_cm*link1_length_cm + distance_from_origin*distance_from_origin - link2_length_cm*link2_length_cm) / (2*link1_length_cm*distance_from_origin))) - atan2(x, -y));
    std::cout << "Calculated angles (radians): Shoulder: " << shoulder_angle << ", Elbow: " << elbow_angle << std::endl;
    // 2. Convert the angles to motor positions (0-4095)
    int motor1_position = 2048 + int((shoulder_angle / (2.0*M_PI)) * 4095);
    int motor2_position = 2048 + int((elbow_angle / (2.0*M_PI)) * 4095);
    int motor3_position = 2048;

    return std::make_tuple(motor1_position, motor2_position, motor3_position);
}