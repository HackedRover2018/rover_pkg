#include <dirent.h>
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>
#include <unistd.h>
#include <rover_pkg/input_msg.h>
#include "ros/ros.h"
#include "serial/serial.h"

#define Pi acos(-1.0)
#define TWITCH_ON 1
#define TWITCH_OFF 0

class CentralControlNode {
 public:
    CentralControlNode(const std::string port, uint32_t baud);
    void rectToPolar(double x, double y, double &r, double &theta);

 private:
    int state_;

    serial::Serial serial_boy_;

    ros::NodeHandle nh_;
    ros::Subscriber input_sub_;
    ros::Subscriber twitch_sub_;

    void inputCallback(const rover_pkg::input_msg::ConstPtr& msg);
    void twitchCallback(const rover_pkg::input_msg::ConstPtr& msg);
};


CentralControlNode::CentralControlNode(const std::string port, uint32_t baud) : 
  state_(TWITCH_OFF),
  serial_boy_(port, baud, serial::Timeout::simpleTimeout(1000)){

  ROS_ERROR("Create input_topic");
  input_sub_ = nh_.subscribe("/input_topic", 10,
      &CentralControlNode::inputCallback, this);

  ROS_ERROR("Create twitch topic");
  twitch_sub_ = nh_.subscribe("/twitch_topic", 10,
      &CentralControlNode::twitchCallback, this);
}

void CentralControlNode::rectToPolar(double x, double y, double &r, double &theta) {
  const double toDegrees = 180.0/Pi;

  r = sqrt((pow(x, 2))+(pow(y, 2)));
  theta = atan(y/x) * toDegrees;

  // Convert theta to the correct quadrant
  if ((x <= 0 && y >= 0) || (x <= 0 && y <= 0)) {
    theta = theta + 180;  // Quad 2 or 3
  } else if (x >= 0 && y < 0) {
    theta = theta + 360;  // Quad 4
  }
}

void CentralControlNode::inputCallback(const rover_pkg::input_msg::ConstPtr& msg) {
  uint8_t buf;
  std::stringstream serial_stream;

  if (msg->switch_state)
    state_ = state_ ^ 1;

  if (state_ == TWITCH_OFF) {

    serial_stream << std::fixed << std::setprecision(5)
      << msg->l_mag << " " << msg->r_mag << "\n";

    const std::string str = serial_stream.str();

    size_t bytes_sent = this->serial_boy_.write(str);
    size_t bytes_read = this->serial_boy_.read(&buf, str.length());

    ROS_INFO("[DRIVE] Bytes Sent [%zu]", bytes_sent);
    ROS_INFO("[DRIVE] Read [%s] [%zu bytes]", &buf, bytes_read);
  }
}


void CentralControlNode::twitchCallback(const rover_pkg::input_msg::ConstPtr& msg) {
  uint8_t buf;
  std::stringstream serial_stream;

  if (state_ == TWITCH_ON) {
    serial_stream << std::fixed << std::setprecision(5)
      << msg->l_mag << " " << msg->r_mag << "\n";

    const std::string str = serial_stream.str();

    size_t bytes_sent = this->serial_boy_.write(str);
    size_t bytes_read = this->serial_boy_.read(&buf, str.length());

    ROS_INFO("[DRIVE] Bytes Sent [%zu]", bytes_sent);
    ROS_INFO("[DRIVE] Read [%s] [%zu bytes]", &buf, bytes_read);
  }
}


int main(int argc, char** argv) {
  
  ROS_INFO("ros::init");
  ros::init(argc, argv, "central_control");

  ROS_INFO("Checking /dev/serial exists");
  DIR* dir = opendir("/dev/serial/");
  if(ENOENT == errno){
    ROS_ERROR("No arduino connected on dev/serial. Please check your connection and restart the ROS structure");
    exit(EXIT_FAILURE);
  }

  const std::string port = "/dev/ttyACM0";
  uint32_t baud = 9600;

  ROS_INFO("Call centralcontrolnode constructor");
  CentralControlNode node(port, baud);

  ros::spin();
  return 0;
}
