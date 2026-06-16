#pragma once
#include <cstdint>
#include <string>
#include <unistd.h>
#include <iostream>
#include <tuple>
#include <algorithm>
#include <cmath>
#include "dynamixel_sdk/dynamixel_sdk.h"

#define position_p_gain_address 84
#define profile_acceleration_address 108
#define profile_velocity_address 112
#define goal_position_address 116
#define present_position_address 132
#define data_length_4byte 4
#define link1_length_cm 27.65
#define link2_length_cm 22.35
#define end_effector_length 7.0

class Arm {
public:
    Arm(const char * port);
    void connect();
    std::tuple<int, int, int> read();
    void write(int target_position1, int target_position2, int target_position3);
    void write(std::string command, double x, double y, int velocity, int acceleration);
    void write(std::string command, double x, double y, int velocity);
    void write(std::string command, double x, double y);
    void write_waypoints(int target_position1, int target_position2, int target_position3, double target_x, double target_y);

    //Helper functions
    std::tuple<double, double, double> calculateSpeeds(int target_position1, int target_position2, int target_position3);
    std::tuple<int, int, int> calculateInverseKinematics(double x, double y);
    std::tuple<double, double> calculateForwardKinematics(int motor1_position, int motor2_position, int motor3_position);
    void toggle_blocking_state(bool block);

private:
    const char * port_;
    
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

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
};