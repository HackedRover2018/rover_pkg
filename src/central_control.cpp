#include <iostream>
#include <sstream>
#include <math.h>
#include <unistd.h>
#include <rover_pkg/input_msg.h>
#include <rover_pkg/drive_msg.h>
#include "ros/ros.h"
#include "serial/serial.h"

#define Pi acos(-1.0)
#define TWITCH_ON 1
#define TWITCH_OFF 0

class CentralControlNode {
 public:
    CentralControlNode();
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


CentralControlNode::CentralControlNode() : 
  state_(TWITCH_OFF),
  serial_boy_("/dev/ttyUSB0", 9600) {

  input_sub_ = nh_.subscribe("/input_topic", 10,
      &CentralControlNode::inputCallback, this);

  twitch_sub_ = nh_.subscribe("/twitch_topic", 10,
      &CentralControlNode::twitchCallback, this);

  if (!serial_boy_.isOpen())
    ROS_INFO("No arduino connected to /dev/ttyUSB0. Please check your connection");
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
  double mag, angle;
  uint8_t buf;
  std::stringstream serial_stream;
  rover_pkg::drive_msg motor_cmd;

  if (msg->state == 1)
    state_ =  state_^1;

  if (state_ == TWITCH_OFF) {
    CentralControlNode::rectToPolar(msg->x_coord, msg->y_coord, mag = 0, angle = 0);
    motor_cmd.magnitude = mag;
    motor_cmd.angle = angle;

    ROS_INFO("[DRIVE] Magnitude  [%f]", motor_cmd.magnitude);
    ROS_INFO("[DRIVE] Polar Angle  [%f]", motor_cmd.angle);

    serial_stream << std::fixed << std::setprecision(5)
      << motor_cmd.magnitude << " " << motor_cmd.angle << "\n";

    const std::string str = serial_stream.str();

    size_t bytes_sent = this->serial_boy_.write(str);
    size_t bytes_read = this->serial_boy_.read(&buf, str.length());

    ROS_INFO("[DRIVE] Bytes Sent [%zu]", bytes_sent);
    ROS_INFO("[DRIVE] Read [%s] [%zu bytes]", &buf, bytes_read);
  }
}

void CentralControlNode::twitchCallback(const rover_pkg::input_msg::ConstPtr& msg) {
  double mag, angle;
  uint8_t buf;
  std::stringstream serial_stream;
  rover_pkg::drive_msg motor_cmd;

  if (state_ == TWITCH_ON) {
    CentralControlNode::rectToPolar(msg->x_coord, msg->y_coord, mag = 0, angle = 0);
    motor_cmd.magnitude = mag;
    motor_cmd.angle = angle;

    ROS_INFO("[DRIVE] Magnitude  [%f]", motor_cmd.magnitude);
    ROS_INFO("[DRIVE] Polar Angle  [%f]", motor_cmd.angle);

    serial_stream << std::fixed << std::setprecision(5)
      << motor_cmd.magnitude << " " << motor_cmd.angle << "\n";

    const std::string str = serial_stream.str();

    size_t bytes_sent = this->serial_boy_.write(str);
    size_t bytes_read = this->serial_boy_.read(&buf, str.length());

    ROS_INFO("[DRIVE] Bytes Sent [%zu]", bytes_sent);
    ROS_INFO("[DRIVE] Read [%s] [%zu bytes]", &buf, bytes_read);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "central_control");
  CentralControlNode node;
  ros::spin();
  return 0;
}
