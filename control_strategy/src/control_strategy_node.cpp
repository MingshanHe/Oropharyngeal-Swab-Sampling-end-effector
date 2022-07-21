#include "control_strategy/control_strategy.h"
#include "control_strategy/control_strategy_params.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_strategy");

    ros::NodeHandle nh;

    if (!nh.getParam("workspace_limits", workspace_limits)) { ROS_ERROR("Couldn't retrieve the workspace_limits."); return -1;}
    if (!nh.getParam("home_pose", home_pose)) { ROS_ERROR("Couldn't retrieve the home pose."); return -1;}
    if (!nh.getParam("work_start_pose", work_start_pose)) { ROS_ERROR("Couldn't retrieve the work start pose."); return -1;}

    Control_Strategy control_strategy(nh, workspace_limits, home_pose, work_start_pose);

    control_strategy.Switch_Controller(0);
    sleep(5);
    control_strategy.Go_Home();
    std::cout<<"Go Home Successfully."<<std::endl;
    sleep(5);
    control_strategy.Go_Work();
    std::cout<<"Go Work Successfully."<<std::endl;
    sleep(5);
    control_strategy.Switch_Controller(3);
    control_strategy.Switch_Wrench(0);
    sleep(5);
    control_strategy.Switch_Controller(2);
    control_strategy.Go_Work();
    sleep(5);
    control_strategy.Switch_Controller(3);
    control_strategy.Switch_Wrench(1);
    sleep(5);
    control_strategy.Switch_Controller(2);
    control_strategy.Go_Work();

    ros::Rate loop_rate(10);


    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}