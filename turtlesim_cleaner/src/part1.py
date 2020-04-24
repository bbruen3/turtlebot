#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def main():
    rospy.init_node('robot_cleaner', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    move(vel_msg, dist_x=2)
    move(vel_msg, dist_y=3)
    move(vel_msg, dist_x=3, isForward=False)
    move(vel_msg, dist_y=2, isForward=False)

def move(vel_msg, dist_x:int=0, dist_y:int=0, isForward:bool=True, speed:int=1):
    # Starts a new node
    #rospy.init_node('robot_cleaner', anonymous=True)
    #velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    #vel_msg = Twist()

    #Checking if the movement is forward or backwards
    if(isForward):
        if dist_x > 0:
            vel_msg.linear.x = abs(speed)
        else:
            vel_msg.linear.x = -abs(speed)
        if dist_y > 0:
            vel_msg.linear.y = abs(speed)
        else:
            vel_msg.linear.y = -abs(speed)
    #Since we are moving just in x-axis
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    while not rospy.is_shutdown():

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_distance = 0

        #Loop to move the turtle in an specified distance
        while(current_distance < distance):
            #Publish the velocity
            velocity_publisher.publish(vel_msg)
            #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
            #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
        #After the loop, stops the robot
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        #Force the robot to stop
        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        #Testing our function
        move()
    except rospy.ROSInterruptException: pass
