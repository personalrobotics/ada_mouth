#!/usr/bin/env python

import rospy
import serial 
from std_msgs.msg import Float64, Float64MultiArray
import math
import struct

DEVICE_ADDRESS = '/dev/ttyACM0' # The device address of the Arduino
CMD_TOPIC = 'mouth_cmd' # The topic that accepts mouth commands
SENSOR_TOPIC = 'mouth_contact' # The topic to publish strain gauge data to
N_SENSORS = 4 # The number of strain gauge sensors

ser = None # A Serial object
sub = None # A subscriber object

# Callback for incoming mouth commands
def cmd_cb(cmd):
  global ser

  # Convert mouth angle to servo angle, then convert from rad to deg
  servo_cmd = int((math.pi/2.0 - cmd.data) * (180.0/math.pi) + 0.5)
  if servo_cmd < 0 or servo_cmd > 255:
    print('Cmd out of range, will skip')
    return

  # Send servo command to the mouth
  packet = b''+struct.pack('!B', servo_cmd)
  ser.write(packet)

# Initialize serial coms and subscriber
def setup():
  global ser, sub

  rospy.init_node('mouth_serial')
 
  ser = serial.Serial(DEVICE_ADDRESS)
  sub = rospy.Subscriber(CMD_TOPIC, Float64, cmd_cb)

def loop():
  # Create publisher for strain gauge commands
  contact_pub = rospy.Publisher(SENSOR_TOPIC, Float64MultiArray, queue_size=1)

  while not rospy.is_shutdown():
    # Block until eexpected number of bytes are received
    sensor_bytes = ser.read(2*N_SENSORS)
    assert(len(sensor_bytes) == 2*N_SENSORS)

    # Check that the received data is valid
    sensor_vals = []
    data_valid = True
    for i in range(0,2*N_SENSORS,2):
      # Get least significant byte
      lsb = struct.unpack('<B', sensor_bytes[i])[0]

      # Get most significant byte
      msb = struct.unpack('<B', sensor_bytes[i+1])[0]
      # Check that both bytes had the expected id in 3 upper bits
      if ((lsb >> 5) != i) or ((msb >> 5) != (i+1)):
        data_valid = False
      # Store the recieved data
      sensor_vals.append(lsb)
      sensor_vals.append(msb)

    if not data_valid:
      # Did not receive all expected data
      # Try to read out remaining data before retrying
      print('Data unsynchronized, realigning...')
      ser.read((2*N_SENSORS-1) - (sensor_vals[-1]>>5))
      continue
        
    msg = Float64MultiArray()

    # De-serialize data
    for i in range(0,2*N_SENSORS,2):
      msg.data.append(((sensor_vals[i+1]&0x01F)<<5)+(sensor_vals[i]&0x01F))

    # Publish strain gauge data
    contact_pub.publish(msg)

if __name__ == '__main__':
  setup()
  loop()
