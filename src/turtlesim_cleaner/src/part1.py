#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
vel_msg = Twist()
turn_msg = Twist()
PI = 3.1415926535897

def main():
    rospy.init_node('robot_cleaner', anonymous=True)
    print("entering move")
    move(dist_x=2, dist_y=0, isForward=True, speed=1)
    print("back from move")
    print("entering rotate")
    rotate(90, False)
    print("back from rotate")
    print("entering y move")
    move(dist_x=3, dist_y=0, isForward=True, speed=1)
    print("back from y move")
    print("rotate cw")
    rotate(90, True)
    print("back from rotate")
    print("move back 3")
    move(dist_x=3, dist_y=0, isForward=False, speed=1)
    print("back from move")
    print("rotate")
    rotate(90, False)
    print("back from rotate")
    print("Move")
    move(dist_x=2, dist_y=0, isForward=False, speed=1)
    print("Back")

def rotate(angle, cw):

    # Receiveing the user's input
    #print("Let's rotate your robot")
    speed = 45

    #Converting from angles to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

    #We wont use linear components
    turn_msg.linear.x=0
    turn_msg.linear.y=0
    turn_msg.linear.z=0
    turn_msg.angular.x = 0
    turn_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if cw:
        turn_msg.angular.z = -abs(angular_speed)
    else:
        turn_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(turn_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)


    #Forcing our robot to stop
    turn_msg.angular.z = 0
    velocity_publisher.publish(turn_msg)
    #rospy.spin()
    return

def move(dist_x, dist_y, isForward, speed):
    global velocity_publisher
    global vel_msg
    # Starts a new node
    #rospy.init_node('robot_cleaner', anonymous=True)
    #velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    #vel_msg = Twist()

    #Checking if the movement is forward or backwards
    if(isForward):
        if dist_x > 0:
            vel_msg.linear.x = abs(speed)
            distance = dist_x
        if dist_y > 0:
            vel_msg.linear.y = abs(speed)
            distance = dist_y
    else:
        if dist_x > 0:
            vel_msg.linear.x = -abs(speed)
            distance = dist_x
        else:
            vel_msg.linear.y = -abs(speed)
            distance = dist_y

    #Since we are moving just in x-axis
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    #while not rospy.is_shutdown():

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
    return

if __name__ == '__main__':
    try:
        #Testing our function
        main()
    except rospy.ROSInterruptException: pass
