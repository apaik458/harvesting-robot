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

constexpr uint16_t kPositionPGainAddress = 84;
constexpr uint16_t kProfileAccelerationAddress = 108;
constexpr uint16_t kProfileVelocityAddress = 112;
constexpr uint16_t kGoalPositionAddress = 116;
constexpr uint16_t kPresentPositionAddress = 132;
constexpr uint16_t kPresentCurrentAddress = 126;
constexpr int kDataLength4Byte = 4;
constexpr double kLink1LengthCm = 27.65;
constexpr double kLink2LengthCm = 22.35;
constexpr double kEndEffectorLength = 7.0;

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

  // Helper functions
  std::tuple<double, double, double> CalculateSpeeds(int target_position1, int target_position2, int target_position3);
  std::tuple<int, int, int> CalculateInverseKinematics(double x, double y);
  std::tuple<double, double> CalculateForwardKinematics(int motor1_position, int motor2_position, int motor3_position);
  void NonBlockingState(bool block);

  // --- Safety / HITL fault testing ---
  // Injects fault scenarios at Arm's raw hardware read points. Pass
  // nullptr (the default) to run with no injection, i.e. normal operation.
  void SetFaultInjector(FaultInjector* injector) { fault_injector_ = injector; }

  // Refreshes the safety-monitoring state (connection status, position,
  // torque) from the hardware. Independent of Read(), which is reserved
  // for motion control, so calling this at monitor rate doesn't add
  // comms traffic to the motion-control hot path. Call once per control
  // loop iteration before consulting FaultMonitor.
  void PollSafetyTelemetry();

  bool IsConnected() const { return connected_; }
  double GetTorque() const { return torque_; }         // Nm, approximate — see PollSafetyTelemetry
  double GetPosition() const { return position_deg_; }  // degrees, shoulder joint (motor1)
  bool IsCommandingMotion() const;

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

  int target_position1_;
  int target_position2_;
  int target_position3_;

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

  // --- Safety / HITL fault testing ---
  FaultInjector* fault_injector_ = nullptr;
  bool connected_ = false;
  double torque_ = 0.0;
  double position_deg_ = 0.0;
  static constexpr double kTorqueConstantNmPerAmp = 1.65;  // a reasoned approximation from datasheet for MX-106
};
