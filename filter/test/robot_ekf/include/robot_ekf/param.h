#pragma once

#include <string>

using std::string;

class Param
{
    
    Param() :
        wheel_frame("wheel"),
        imu_frame("imu"),
        output_frame("odom"),
        base_frame("base"){}

public:

    string wheel_frame;
    string imu_frame;
    string output_frame;
    string base_frame;

    static Param& getInstance()
    {
        static Param p;
        return p;
    }

};