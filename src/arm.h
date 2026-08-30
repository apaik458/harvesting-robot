#pragma once
#include <cstdint>
#include <string>
#include <unistd.h>
#include <iostream>
#include <tuple>
#include <algorithm>
#include <cmath>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "fault_injector.h"

// Parameters from Dynamixel datasheet
constexpr uint16_t PositionPGainAddress = 84;
constexpr uint16_t ProfileAccelerationAddress = 108;
constexpr uint16_t ProfileVelocityAddress = 112;
constexpr uint16_t GoalPositionAddress = 116;
constexpr uint16_t PresentPositionAddress = 132;
constexpr uint16_t PresentCurrentAddress = 126;
constexpr int DataLength4Byte = 4;

// Physical attributes of the robot
constexpr double Link1LengthCm = 27.65;
constexpr double Link2LengthCm = 22.35;
constexpr double EndEffectorLength = 7.0;

class Arm {
 public:
  Arm(const char* port);
  void Connect();
  std::tuple<int, int, int> Read();
  void Write(int target_position1, int target_position2, int target_position3);
  void Write(std::string command, double x, double y, int velocity, int acceleration);
  void Write(std::string command, double x, double y, int velocity);
  void Write(std::string command, double x, double y);
  void WriteWaypoints(int target_position1, int target_position2, int target_position3, double target_x, double target_y);
  void NonBlockingState(bool block);
  void SetFaultInjector(FaultInjector* injector);
  void PollSafetyTelemetry();
  bool IsConnected() const;
  double GetMaxTorque() const;
  std::tuple<bool, bool, bool> GetMotorConnections() const;
  std::tuple<double, double, double> GetMotorTorques() const;

  // Helper functions
  std::tuple<double, double, double> CalculateSpeeds(int target_position1, int target_position2, int target_position3);
  std::tuple<int, int, int> CalculateInverseKinematics(double x, double y);
  std::tuple<double, double> CalculateForwardKinematics(int motor1_position, int motor2_position, int motor3_position);

 private:
  const char* port_;

  dynamixel::PortHandler* port_handler_;
  dynamixel::PacketHandler* packet_handler_;

  uint8_t dxl_id1_;
  uint8_t dxl_id2_;
  uint8_t dxl_id3_;

  int motor1_current_position_;
  int motor2_current_position_;
  int motor3_current_position_;

  int max_velocity_;
  int max_acceleration_;

  bool non_blocking_;

  int move_accuracy_threshold_;

  float current_x_;
  float current_y_;

  uint8_t dxl_error_;
  int dxl_comm_result_;
  bool dxl_addparam_result_;
  bool dxl_getdata_result_;

  int waypoints_number_;
  int large_movement_threshold_;

  FaultInjector* fault_injector_;
  bool connected_;
  double torque_;
  bool motor_connected_[3];
  double motor_torque_[3];
  double torque_constant_Nm_per_amp;  // a reasoned approximation from datasheet for MX-106
};
