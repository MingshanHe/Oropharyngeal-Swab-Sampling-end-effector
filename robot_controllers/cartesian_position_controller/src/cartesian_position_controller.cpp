/*
 * @Author: MingshanHe 
 * @Date: 2021-12-05 04:09:41 
 * @Last Modified by: MingshanHe
 * @Last Modified time: 2021-12-05 04:18:32
 * @Licence: MIT Licence
 */
#include <pluginlib/class_list_macros.h>
#include "cartesian_position_controller/kinematics_base.h"
#include "cartesian_position_controller/cartesian_position_controller.h"
#include "kdl_conversions/kdl_msg.h"

namespace cartesian_position_controller
{
/** \brief Initialize the kinematic chain for kinematics-based computation.
 *
 */
bool Cartesian_Position_Controller::init(
    hardware_interface::PositionJointInterface *robot, ros::NodeHandle &n) {

  // KDL
  kinematics_base::Kinematics_Base::init(robot, n);

  ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv_givens(this->kdl_chain_));
  ik_pos_solver_.reset(new KDL::ChainIkSolverPos_LMA(this->kdl_chain_));
  fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(this->kdl_chain_));
  fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(this->kdl_chain_));

  // get param
  if (!n.getParam("publish_rate", publish_rate_)){
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
  }

  if (!n.getParam("max_velocity", max_velocity_)){
      ROS_ERROR("Parameter 'max_velocity' not set");
      return false;
  }

  if (!n.getParam("step_position", step_position_)){
      ROS_ERROR("Parameter 'step_position' not set");
      return false;
  }

  if (!n.getParam("step_velocity", step_velocity_)){
      ROS_ERROR("Parameter 'step_velocity' not set");
      return false;
  }

  realtime_pub_.reset(new realtime_tools::RealtimePublisher
    <cartesian_state_msgs::PoseTwist>(n, "ee_state", 4));


  // Topics
  sub_command_ = n.subscribe("command_cart_pos", 5,
    &Cartesian_Position_Controller::command_cart_pos, this,
    ros::TransportHints().reliable().tcpNoDelay());

  // Variable init
  this->joint_state_.resize(this->kdl_chain_.getNrOfJoints());
  this->joint_effort_.resize(this->kdl_chain_.getNrOfJoints());
  Jnt_Vel_Cmd_.resize(this->kdl_chain_.getNrOfJoints());
  Jnt_Pos_Cmd_.resize(this->kdl_chain_.getNrOfJoints());

  End_Pos_.p.Zero();
  End_Pos_.M.Identity();
  End_Vel_.p.Zero();
  End_Vel_.M.Identity();
  pre_e = 0.0;
  return true;
}

/** \brief This is called from within the realtime thread just before the
 * first call to \ref update
 *
 * \param time The current time
 */
void Cartesian_Position_Controller::starting(const ros::Time& time){
  for(std::size_t i=0; i < this->joint_handles_.size(); i++) {
    Jnt_Vel_Cmd_(i) = 0.0;
    Jnt_Pos_Cmd_(i) = 0.0;
    this->joint_state_.q(i)     = 0.0;
    this->joint_state_.qdot(i)  = 0.0;
    this->joint_effort_(i)    = 0.0;
    this->joint_state_.q(i)         = this->joint_handles_[i].getPosition();
  }
  fk_pos_solver_->JntToCart(this->joint_state_.q, End_Pos_Cmd_);
  last_publish_time_ = time;
}

/*!
 * \brief Issues commands to the joint. Should be called at regular intervals
 */
void Cartesian_Position_Controller::update(const ros::Time& time, const ros::Duration& period) {
  // Get joint positions
  for(std::size_t i=0; i < this->joint_handles_.size(); i++)
  {
    this->joint_state_.q(i)         = this->joint_handles_[i].getPosition();
    this->joint_state_.qdot(i)      = this->joint_handles_[i].getVelocity();
    this->joint_effort_(i)        = this->joint_handles_[i].getEffort();
  }

  // Forward kinematics
  fk_vel_solver_->JntToCart(this->joint_state_, End_Vel_);
  fk_pos_solver_->JntToCart(this->joint_state_.q, End_Pos_);

  // ik_vel_solver_->CartToJnt(this->joint_state_.q, End_Vel_Cmd_, Jnt_Vel_Cmd_);
  // double  max_time = 0;
  // for(std::size_t i=0; i < this->joint_handles_.size(); i++)
  // {
  //   double time = fabs(this->joint_state_.q(i) - Jnt_Pos_Cmd_(i))/step_position_[i];
  //   if (time > max_time)
  //   {
  //     max_time = time;
  //   }
  // }
  // End_Pos_Vector[0] = End_Pos_.p[0] + (End_Pos_Cmd_.p[0] - End_Pos_.p[0])/max_time;
  // End_Pos_Vector[1] = End_Pos_.p[1] + (End_Pos_Cmd_.p[1] - End_Pos_.p[1])/max_time;
  // End_Pos_Vector[2] = End_Pos_.p[2] + (End_Pos_Cmd_.p[2] - End_Pos_.p[2])/max_time;
  // double starting[4];
  // double ending[4];
  // End_Pos_.M.GetQuaternion(starting[0],starting[1],starting[2],starting[3]);
  // End_Pos_Cmd_.M.GetQuaternion(ending[0],ending[1],ending[2],ending[3]);
  // float cosa = starting[0]*ending[0] + starting[1]*ending[1] + starting[2]*ending[2] + starting[3]*ending[3];
  // if ( cosa < 0.0f ) 
  // {
  //     ending[0] = -ending[0];
  //     ending[1] = -ending[1];
  //     ending[2] = -ending[2];
  //     ending[3] = -ending[3];
  //     cosa = -cosa;
  // }
  
  // float k0, k1;
  // if ( cosa > 0.9995f ) 
  // {
  //     k0 = 1.0f - (1/max_time);
  //     k1 = (1/max_time);
  // }
  // else 
  // {
  //     float sina = sqrt( 1.0f - cosa*cosa );
  //     float a = atan2( sina, cosa );
  //     k0 = sin((1.0f - (1/max_time))*a)  / sina;
  //     k1 = sin( (1/max_time)*a) / sina;
  // }
  // double x, y, z, w;
  // x = starting[0]*k0 + ending[0]*k1;
  // y = starting[1]*k0 + ending[1]*k1;
  // z = starting[2]*k0 + ending[2]*k1;
  // w = starting[3]*k0 + ending[3]*k1;

  // End_Pos_Cmd_.p = End_Pos_Vector;
  // End_Pos_Cmd_.M = End_Pos_Rotation.Quaternion(x,y,z,w);

  ik_pos_solver_->CartToJnt(this->joint_state_.q, End_Pos_Cmd_, Jnt_Pos_Cmd_);
  // std::cout<<"Jnt_Pos_Cmd_: "<<Jnt_Pos_Cmd_(0)<<","<<Jnt_Pos_Cmd_(1)<<","<<Jnt_Pos_Cmd_(2)<<","<<Jnt_Pos_Cmd_(3)<<","<<Jnt_Pos_Cmd_(4)<<","<<Jnt_Pos_Cmd_(5)<<std::endl;
  writePositionCommands(period);
  
  // Limit rate of publishing
  if (publish_rate_ > 0.0 && last_publish_time_
       + ros::Duration(1.0/publish_rate_) < time) {

    // try to publish
    if (realtime_pub_->trylock()) {
      last_publish_time_ = last_publish_time_
                           + ros::Duration(1.0/publish_rate_);
      // populate message
      realtime_pub_->msg_.header.stamp = time;
      tf::poseKDLToMsg(End_Pos_, realtime_pub_->msg_.pose);
      tf::twistKDLToMsg(End_Vel_.GetTwist(), realtime_pub_->msg_.twist);

      realtime_pub_->unlockAndPublish();
    }
  }
}

/*!
 * \brief Subscriber's callback: copies twist commands
 */
void Cartesian_Position_Controller::command_cart_pos(const geometry_msgs::PoseConstPtr &msg) {
  double x,y,z,w;
  x = msg->orientation.x;
  y = msg->orientation.y;
  z = msg->orientation.z;
  w = msg->orientation.w;
  // End_Pos_Rotation.GetQuaternion(x,y,z,w);
  // std::cout<<End_Pos_Rotation<<std::endl;

  End_Pos_Vector[0] = msg->position.x;
  End_Pos_Vector[1] = msg->position.y;
  End_Pos_Vector[2] = msg->position.z;

  End_Pos_Cmd_.p = End_Pos_Vector;
  End_Pos_Cmd_.M = End_Pos_Rotation.Quaternion(x,y,z,w);
}


/********************************************/
/**FUNCTIONS OF INSTANCES OF THE BASE CLASS**/
/********************************************/

/** \brief write the desired velocity command in the hardware interface input
 * for a VelocityJointInterface
 * \param period The duration of an update cycle
 */

void Cartesian_Position_Controller::writePositionCommands(
                                    const ros::Duration& period)  {
    for(std::size_t i = 0; i < this->joint_handles_.size(); i++){
      // Method1:
      // double pos_delta;
      // double Kp = 0.05;
      // double Kd = 0.00;
      if((Jnt_Pos_Cmd_(i)-this->joint_state_.q(i)) > step_position_[i])
      {
        this->joint_handles_[i].setCommand(this->joint_state_.q(i) +step_position_[i]);
      }
      else if(-(Jnt_Pos_Cmd_(i)-this->joint_state_.q(i)) > step_position_[i])
      {
        this->joint_handles_[i].setCommand(this->joint_state_.q(i) - step_position_[i]);
      }
      else
      {
        this->joint_handles_[i].setCommand(this->joint_state_.q(i) + (Jnt_Pos_Cmd_(i)-this->joint_state_.q(i)));
      }
      // Method2:
      // this->joint_handles_[i].setCommand(this->joint_state_.q(i)
      //                                 + Jnt_Vel_Cmd_(i)*period.toSec());
    }
}

} // controller_interface namespace

// Register controllers with the PLUGINLIB_EXPORT_CLASS macro to enable dynamic
// loading with the controller manager
PLUGINLIB_EXPORT_CLASS(cartesian_position_controller::Cartesian_Position_Controller,
                       controller_interface::ControllerBase)
