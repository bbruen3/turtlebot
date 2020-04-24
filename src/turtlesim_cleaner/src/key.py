#!/usr/bin/env python
import rospy
from geometry_msgs.msg  import Twist
from turtlesim.msg import Pose
from math import pow,atan2,sqrt,pi

def cap_to_pi(angle):
    if angle <= -pi: return 2*pi+angle
    if angle > pi: return -2*pi+angle
    return angle

class turtlebot():
    def __init__(self):
        # Creating our node, publisher and subscriber
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.callback)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    # Callback function implementing the pose value received
    def callback(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def get_distance(self, goal_x, goal_y):
        distance = sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
        return distance

    # Will move turtlebot to specified position from the current position
    # The speed of the turtle is proportional to the distance to the goal
    def move2goal(self, x, y, distance_tolerance):
        goal_pose = Pose()
        goal_pose.x = x
        goal_pose.y = y
        vel_msg = Twist()

        # Keep moving until we reach the goal
        while sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2)) >= distance_tolerance:

            #Porportional Controller
            #linear velocity in the x-axis:
            vel_msg.linear.x = 1.5 * sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            #angular velocity in the z-axis:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 4 * cap_to_pi(atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x) - self.pose.theta)

            #Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Stopping our robot after the movement is over
        vel_msg.linear.x  = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # print("I'm done. Please, kill me now!")
        # rospy.spin()

if __name__ == '__main__':
    try:
        x = turtlebot()
        # The middle of the space is in (5.5, 5.5)
        x.move2goal( 1  *.7 + 5.5, 6  *.7 + 5.5, 0.1)
        x.move2goal(-1  *.7 + 5.5, 0  *.7 + 5.5, 0.1)
        x.move2goal(-1  *.7 + 5.5, 5  *.7 + 5.5, 0.1)
        x.move2goal( 1  *.7 + 5.5, 5  *.7 + 5.5, 0.1)
        x.move2goal( 1  *.7 + 5.5, 6  *.7 + 5.5, 0.1)

    except rospy.ROSInterruptException: pass

