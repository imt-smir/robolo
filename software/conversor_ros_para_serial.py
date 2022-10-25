#!/usr/bin/env python3

import serial
import rospy
from geometry_msgs.msg import Twist

ser = serial.Serial('/dev/ttyUSB0', 57600, timeout=1) 

def callback(data):
    ser.write(b'ssss')


    imax = 3
    if(data.linear.x < 0.0):
        imax = 4
    for i in range(imax):
        ser.write(bytes(str(data.linear.x)[i], 'utf-8'))
    
    print(str(data.linear.x)[0:imax], end='\t');


    imax = 3
    if(data.angular.z < 0.0):
        imax = 4
    for i in range(imax):
        ser.write(bytes(str(data.angular.z)[i], 'utf-8'))

    print(str(data.angular.z)[0:imax]);


def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == "__main__":
    main()