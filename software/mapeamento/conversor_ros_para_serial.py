#!/usr/bin/env python3

#teoria:    <char_inicio><data><check_sum><char_finalização>
#prática:   <\x05><<vel_linear><vel_angular>><len(data)><\x04>
#exemplo:   "\x051.0-4.37\x04"


import serial
import rospy
from geometry_msgs.msg import Twist

serial_input = serial.Serial('/dev/ttyUSB0', 57600, timeout=1)


def sending_vel_data(vel:float) -> int:
    imax = 3
    if(vel < 0.0):
        imax = 4
    for i in range(imax):
        serial_input.write(bytes(str(vel)[i], 'utf-8'))    
    print(str(vel)[0:imax], end='')    
   
    return imax


def callback(data:Twist) -> None:
    serial_input.write(b'\x05')
    print('\x05', end='')    

    check_sum = 0
    check_sum += sending_vel_data(data.linear.x)
    check_sum += sending_vel_data(data.angular.z)


    serial_input.write(bytes(str(check_sum), 'utf-8'))
    print(check_sum, end='')    
    serial_input.write(b'\x04')
    print('\x04', end='\n')    




def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == "__main__":
    main()