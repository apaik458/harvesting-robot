#pragma once
#include <cstdint>
#include <string>
#include <unistd.h>
#include <iostream>
#include <tuple>
#include <algorithm>
#include <cmath>
#include "dynamixel_sdk/dynamixel_sdk.h"

#define profile_velocity_address 112
#define goal_position_address 116
#define present_position_address 132
#define data_length_4byte 4

#define link1_length_cm 27.65
#define link2_length_cm 22.35

class Arm {
public:
    Arm(const char * port);
    void connect();
    std::tuple<int, int, int> read();
    void write(int target_position1, int target_position2, int target_position3);
    void write(std::string command, float x, float y);

    //Helper functions
    std::tuple<int, int, int> calculateSpeeds(int target_position1, int target_position2, int target_position3);
    std::tuple<int, int, int> calculateInverseKinematics(float x, float y);

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

    float current_x_;
    float current_y_;

    uint8_t dxl_error_;
    int dxl_comm_result_;
    bool dxl_addparam_result_;
    bool dxl_getdata_result_;
};