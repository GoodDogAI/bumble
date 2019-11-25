#!/usr/bin/env python
import rospy
import sys

from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

def move():
    # Starts a new node
    rospy.init_node('joystick', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Calibrate the motors
    rospy.wait_for_service("/odrive/calibrate_motors")
    calibrate_motors = rospy.ServiceProxy("/odrive/calibrate_motors", Trigger)
    print("calibrating motors", calibrate_motors())

    # Engage the motors
    rospy.wait_for_service("/odrive/engage_motors")
    engage_motors = rospy.ServiceProxy("/odrive/engage_motors", Trigger)

    print("engaging motors", engage_motors())

    # #Receiveing the user's input
    # print("Let's move your robot")
    # speed = input("Input your speed:")
    # distance = input("Type your distance:")
    # isForward = input("Foward?: ")#True or False
    #
    # #Checking if the movement is forward or backwards
    # if(isForward):
    #     vel_msg.linear.x = abs(speed)
    # else:
    #     vel_msg.linear.x = -abs(speed)
    # #Since we are moving just in x-axis
    # vel_msg.linear.y = 0
    # vel_msg.linear.z = 0
    # vel_msg.angular.x = 0
    # vel_msg.angular.y = 0
    # vel_msg.angular.z = 0
    #
    while not rospy.is_shutdown():
        print(sys.stdin.read(1))

        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass