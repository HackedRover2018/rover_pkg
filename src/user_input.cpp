#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <rover_pkg/input_msg.h>
#include <iostream>
#include <unistd.h>

#define L_AXIS 1
#define R_AXIS 3
#define TRI_BUT 12

class UserInputNode{
  public:
    UserInputNode();

  private:
    int xscale_, yscale_;

    ros::NodeHandle nh_;
    ros::Subscriber ps3_sub_;
    ros::Publisher cmd_pub_;

    void ps3CmdCallback(const sensor_msgs::Joy::ConstPtr& ps3_msg);
};

UserInputNode::UserInputNode():
  yscale_(1),
  xscale_(1){
  
  cmd_pub_ = nh_.advertise<rover_pkg::input_msg>("/input_topic", 10);
  ps3_sub_ = nh_.subscribe("joy", 10, &UserInputNode::ps3CmdCallback, this);
  
}

void UserInputNode::ps3CmdCallback(const sensor_msgs::Joy::ConstPtr& msg){

  //ROS_INFO("PS3 MSG RECEIVED");

  rover_pkg::input_msg polar_msg;
  
  polar_msg.switch_state = 0;
  polar_msg.r_mag = xscale_*msg->axes[R_AXIS];
  polar_msg.l_mag = yscale_*msg->axes[L_AXIS];
  ROS_INFO("r_mag = %f", polar_msg.r_mag);
  ROS_INFO("l_mag = %f", polar_msg.l_mag);

  if(msg->buttons[TRI_BUT]){
    polar_msg.switch_state = 1;  
  }

  cmd_pub_.publish(polar_msg);

}

int main(int argc, char** argv){

  ros::init(argc, argv, "user_input");

  UserInputNode user_input_node;

  ros::spin();

}
