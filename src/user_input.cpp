#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <iostream>
#include <unistd.h>

class UserInputNode{
  public:
    UserInputNode();

  private:
    int xin_axis_, yin_axis_, tri_button_;
    int xscale_, yscale_;

    ros::NodeHandle nh_;
    ros::Subscriber ps3_sub_;
    ros::Publisher cmd_pub_;

    void ps3CmdCallback(const sensor_msgs::Joy::ConstPtr& ps3_msg);
};

UserInputNode::UserInputNode():
  yin_axis_(1),
  xin_axis_(2),
  tri_button_(12),
  yscale_(1),
  xscale_(-1){
  
  //cmdpub_ = nh_.advertise(/*TODO CREATE CUSTOM MSG AND ADD IT HERE*/, "/input", 10);
  ps3_sub_ = nh_.subscribe("joy", 10, &UserInputNode::ps3CmdCallback, this);
  
}

void UserInputNode::ps3CmdCallback(const sensor_msgs::Joy::ConstPtr& ps3_msg){

  ROS_INFO("PS3 MSG RECEIVED");
}

int main(int argc, char** argv){

  ros::init(argc, argv, "user_input");

  UserInputNode user_input_node;

  ros::spin();

}
