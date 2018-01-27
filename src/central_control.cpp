#include <iostream>
#include <sstream>
#include <math.h>
#include <unistd.h>
#include <rover_pkg/input_msg>
#include <rover_pkg/drive_msg>
#include "ros/ros.h"
#include "serail/serial.h"

class CentralControlNode {
 public:
    CentralControlNode();
    void rectToPolar(double x, double y, double &r, double &theta);

 private:
      ros::NodeHandle ng_;
      ros::Subsriber input_sub_;
      /* ros::Publisher drive_pub_; */

      void inputCallback(const rover_pkg::input_msg::ConstPtr& msg);
};


CentralControlNode::CentralControlNode() {
  /* drive_pub_ = nh_.advertise<rover_pkg::drive_msgs>("/drive_topic", 10); */

  inpout_sub_ = nh_.subscribe("/input_topic", 10,
      &CentralControlNode::inputCallback, this);
}

CentralControlNode::rectToPolar(double x, douvle y, douvle &r, double &theta) {
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

void inputCallback(const rover_pkg::input_msg::ConstPtr& msg) {
  double mag, angle;
  uint8_t buf;
  std::stringstream serial_stream;
  rover_pkg::drive_msg motor_cmd;

  CentralControlNode::rectToPolar(msg->x_coor, msg->y_coor, mag = 0, angle = 0);
  motor_cmd.magnitude = mag;
  motor_cmd.angle = angle;

  ROS_INFO("[DRIVE] Magnitude  [%f]", motor_cmd.magnitude);
  ROS_INFO("[DRIVE] Polar Angle  [%f]", motor_cmd.polar_angle);

  serial_stream << std::fixed << std::setprecision(5)
    << motor_cmd.magnitude << " " << motor_cmd.angle << "\n";

  const std::string str = serial_stream.str();

  size_t bytes_sent = this->my_serial.write(str);
  size_t bytes_read = this->my_serial.read(&buf, str.length());

  ROS_INFO("[DRIVE] Bytes Sent [%zu]", bytes_sent);
  ROS_INFO("[DRIVE] Read [%s] [%zu bytes]", &buf, bytes_read)
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "central_control");
  CentralControlNode node;
  ros::spin();
  return 0;
}
