#!/usr/bin/env python
#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import os

from matplotlib import pyplot as plt
import numpy as np

fig = plt.figure()
ax = plt.axes()
ax.set_xlim([0, 10])
ax.set_ylim([0, 10])

class TurtleBot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',
                                                  Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
                                                Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def define_obstacle(self):
        self.obstacle_pose = Pose()

        # Get the input from the user.
        self.obstacle_pose.x = input("Set the circular obstacle center x: ")
        self.obstacle_pose.y = input("Set the circular obstacle center y: ")

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        self.obstacle_radius = input("Set the circular obstacle radius: ")
        
        # plot obstacle
        circle=plt.Circle((self.obstacle_pose.x,self.obstacle_pose.y),self.obstacle_radius, color='r')
        fig2 = plt.gcf()
        ax = fig2.gca()
        ax.add_artist(circle)
        
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))-self.obstacle_radius

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def stop2obstacle(self):
        vel_msg = Twist()
        self.define_obstacle()
        
        while self.euclidean_distance(self.obstacle_pose) >= self.obstacle_radius:
            # arrow representing orientation of turtle
            arrow = np.array([1, 0, 0]).T
            R = np.array([[np.cos(self.pose.theta), -np.sin(self.pose.theta), 0], [np.sin(self.pose.theta), np.cos(self.pose.theta), 0], [0, 0, 1]])
            arrow_new = R.dot(arrow)
            
            ax.arrow(self.pose.x, self.pose.y, arrow_new[0], arrow_new[1], head_width=0.3, head_length=0.1, fc='g', ec='g')
            plt.pause(0.05)
            
            print ("Current position: ",self.pose.x,self.pose.y)
            print ("Distance to obstacle: ",self.euclidean_distance(self.obstacle_pose))
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
     
        node = "/teleop_key"
        os.system("rosnode kill "+ node)
        # If we press control + C, the node will stop.
        print ("Ctrl+C to stop the code...")
        rospy.spin()

    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = input("Set your x goal: ")
        goal_pose.y = input("Set your y goal: ")

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = input("Set your tolerance: ")

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
	
        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.stop2obstacle()

    except rospy.ROSInterruptException:
        pass
