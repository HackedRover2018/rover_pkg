#!/usr/bin/env python

import rospy
import urllib

from rover_pkg.msg import input_msg


class TwitchInputNode():
  def __init__(self):
  
    # Initialize the publisher
    pub = rospy.Publisher('twitch_topic', input_msg)

    # Create a rate object
    rate = rospy.Rate(2) # 2Hz

    while not rospy.is_shutdown():
      # Get message from url
      req = urllib.urlopen("http://ec2-52-53-222-168.us-west-1.compute.amazonaws.com:8080")
      twitch_str = req.read()

      msg = input_msg()
      msg.x_coord = ''
      msg.y_coord = ''

      rospy.loginfo(msg)

      # Publish message to topic
      pub.publish(msg)

      # Goodnight my sweet prince
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node('twitch_input_node')
  try:
    node = TwitchInputNode()
  except rospy.ROSInterruptException: pass
