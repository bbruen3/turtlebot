#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

def move(dist, f):
    # Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    
    #Receiveing the user's input
    speed = 1
    distance = dist
    isForward = f
    
    #Checking if the movement is forward or backwards
    if(isForward):
        vel_msg.linear.x = 1
    else:
        vel_msg.linear.x = -1
    #Since we are moving just in x-axis
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    #Setting the current time for distance calculus
    t0 = float(rospy.Time.now().to_sec())
    current_distance = 0

    #Loop to move the turtle in an specified distance
    while(current_distance < distance):
    #Publish the velocity
        velocity_publisher.publish(vel_msg)
        #Takes actual time to velocity calculus
        t1=float(rospy.Time.now().to_sec())
        #Calculates distancePoseStamped
        current_distance= speed*(t1-t0)
    #After the loop, stops the robot
    vel_msg.linear.x = 0
    #Force the robot to stop
    velocity_publisher.publish(vel_msg)

def rotate(deg, ang, cl):

    #Starts a new node
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    # Receiveing the user's input
    speed = 10
    angle = ang
    clockwise = cl

    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    if deg == 1:
        relative_angle = angle*2*PI/360
    else:
        relative_angle = angle

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)


    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    

if __name__ == '__main__':
    try:
        #Testing our function
        move(2, 1)
	rotate(1, 90, False)
	move(3, 1)
	rotate(1, 90, True)
	move(3, 0)
	rotate(1, 90, False)
	move(2, 0)
    except rospy.ROSInterruptException: pass

