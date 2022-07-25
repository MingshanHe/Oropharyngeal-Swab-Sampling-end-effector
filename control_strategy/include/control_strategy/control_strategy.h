#ifndef CONTROL_STRATEGY_H
#define CONTROL_STRATEGY_H
#include "ros/ros.h"
#include <iostream>
#include <string>

#include "controller_manager_msgs/SwitchController.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Wrench.h"
#include "cartesian_state_msgs/PoseTwist.h"
#include "control_msgs/GripperCommandActionGoal.h"
#include "control_msgs/GripperCommand.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/opencv.hpp"

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>

#include "utils/logger.h"

#include "utils/filter.h"

#define SwitchController_Topic      "/controller_manager/switch_controller"
#define CartesianPose_Topic         "/cartesian_position_controller/command_cart_pos"
#define CartesianTwist_Topic        "/cartesian_velocity_controller/command_cart_vel"
#define CartesianState_Topic        "/cartesian_velocity_controller/ee_state"
#define ImageWrench_Topic           "/twist"
#define FakeWrench_Topic            "/fake_wrench"

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

class Control_Strategy
{
private:
    /* data */
public:
    Control_Strategy(const ros::NodeHandle &nh_,
                        std::vector<double> workspace_limits_,
                        std::vector<double> home_pose_,
                        std::vector<double> work_start_pose_);
    ~Control_Strategy(){};
public:
    void Switch_Controller(const int &cognition);
    void Switch_Wrench(const int &cognition);
    void Go_Home(void);
    void Go_Work(void);
    void Go(std::vector<double> Position);
    void Gitter();
public:
    void Cartesian_State_Cb(const cartesian_state_msgs::PoseTwistConstPtr &msg);
    void ImageWrench_Cb(const geometry_msgs::Wrench &msg);
private:
    ros::NodeHandle                                         nh;
    ros::Publisher                                          Cartesian_Pose_Pub;
    ros::Publisher                                          Cartesian_Twist_Pub;
    ros::Publisher                                          Predict_IMG_Pub;
    ros::Publisher                                          Gripper_Pub;
    ros::Publisher                                          Wrench_Pub;
    ros::Subscriber                                         Cartesian_State_Sub;
    ros::Subscriber                                         Wrench_Sub;
    ros::ServiceClient                                      switch_controller_client;
    controller_manager_msgs::SwitchController               switch_controller_srv;
    std::vector<std::string, std::allocator<std::string>>   start_controllers;
    std::vector<std::string, std::allocator<std::string>>   stop_controllers;

private:
    double                                                  force_x;
    double                                                  force_y;
    double                                                  force_z;


    Vector6d                                                workspace_limits;
    Vector7d                                                home_pose;
    Vector7d                                                work_start_pose;


    Vector7d                                                Cartesian_State;
    double                                                  pos_x, pos_y, pos_z;
    double                                                  ori_x, ori_y, ori_z, ori_w;

private:
    Filter                                                  filter;
    Logger                                                  logger;
};



#endif