#ifndef LOGGER_H
#define LOGGER_H
#include "ros/ros.h"
#include <sensor_msgs/Image.h>

#include <iostream>
#include <fstream>
#include <ctime>

class Logger
{
private:
    time_t          now_time;
    std::string     command;
    std::string     folderPath;
    std::string     packagePath;
    std::ofstream   wrenchFile;
public:
    Logger();
    ~Logger();
public:
    void save_wrench(double force_x, double force_y, double force_z, double torque_x, double torque_y, double torque_z);
};
#endif