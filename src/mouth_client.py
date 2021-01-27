#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

CMD_TARGET = 1.0
CMD_TOPIC = 'mouth_cmd'

if __name__ == '__main__':
  rospy.init_node('mouth_client', anonymous=True)
  cmd_pub = rospy.Publisher(CMD_TOPIC, Float64, queue_size=1)
  rate = rospy.Rate(1)

  while not rospy.is_shutdown():
    msg = Float64()
    msg.data = CMD_TARGET
    cmd_pub.publish(msg)
    rate.sleep()
