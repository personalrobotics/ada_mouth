#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

CMD_TARGET = 1.0 # Opens the mouth
# CMD_TARGET = 0.1 # Closes the mouth
CMD_TOPIC = 'mouth_cmd' # The topic to publish the mouth cmd to

# This test script commands the mouth to change its position to CMD_TARGET
if __name__ == '__main__':
  rospy.init_node('mouth_client', anonymous=True)

  # Create a publisher of commands
  cmd_pub = rospy.Publisher(CMD_TOPIC, Float64, queue_size=1)
  rate = rospy.Rate(1)

  # Send the command
  while not rospy.is_shutdown():
    msg = Float64()
    msg.data = CMD_TARGET
    cmd_pub.publish(msg)
    rate.sleep()
