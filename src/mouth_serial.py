#!/usr/bin/env python

import rospy
import serial 
from std_msgs.msg import Float64, Float64MultiArray
import math
import struct

DEVICE_ADDRESS = '/dev/ttyACM0'
CMD_TOPIC = 'mouth_cmd'
SENSOR_TOPIC = 'mouth_contact'
N_SENSORS = 4

ser = None
sub = None

def cmd_cb(cmd):
  global ser

  servo_cmd = int((math.pi/2.0 - cmd.data) * (180.0/math.pi) + 0.5)
  if servo_cmd < 0 or servo_cmd > 255:
    print('Cmd out of range, will skip')
    return

  packet = b''+struct.pack('!B', servo_cmd)
  ser.write(packet)

def setup():
  global ser, sub

  rospy.init_node('mouth_serial')
 
  ser = serial.Serial(DEVICE_ADDRESS)
  sub = rospy.Subscriber(CMD_TOPIC, Float64, cmd_cb)

def loop():
  contact_pub = rospy.Publisher(SENSOR_TOPIC, Float64MultiArray, queue_size=1)

  while not rospy.is_shutdown():

    sensor_bytes = ser.read(2*N_SENSORS)
    assert(len(sensor_bytes) == 2*N_SENSORS)
    sensor_vals = []
    data_valid = True
    for i in range(0,2*N_SENSORS,2):
      lsb = struct.unpack('<B', sensor_bytes[i])[0]
      msb = struct.unpack('<B', sensor_bytes[i+1])[0]
      if ((lsb >> 5) != i) or ((msb >> 5) != (i+1)):
        data_valid = False
      sensor_vals.append(lsb)
      sensor_vals.append(msb)

    if not data_valid:
      print('Data unsynchronized, realigning...')
      ser.read((2*N_SENSORS-1) - (sensor_vals[-1]>>5))
      continue
        
    msg = Float64MultiArray()

    for i in range(0,2*N_SENSORS,2):
      msg.data.append(((sensor_vals[i+1]&0x01F)<<5)+(sensor_vals[i]&0x01F))

    contact_pub.publish(msg)

if __name__ == '__main__':
  setup()
  loop()
