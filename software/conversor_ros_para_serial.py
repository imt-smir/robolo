#!/usr/bin/env python3

import serial
import rospy
from geometry_msgs.msg import Twist

serial_input = serial.Serial('/dev/ttyUSB0', 57600, timeout=1) 

def sending_data(vel:float) -> None:
    imax = 3
    if(vel < 0.0):
        imax = 4
    for i in range(imax):
        serial_input.write(bytes(str(vel)[i], 'utf-8'))
    
    print(str(vel)[0:imax], end='\t')

def callback(data:Twist) -> None:
    
    serial_input.write(b'ssss')

    sending_data(data.linear.x)
    sending_data(data.angular.z)
    


def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == "__main__":
    main()