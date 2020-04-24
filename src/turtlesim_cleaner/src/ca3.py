#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

PI = 3.1415926535897

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

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
    
    def lateral_move(self, isForward, dist_x, speed):
        vel_msg = Twist()
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
            self.velocity_publisher.publish(vel_msg)
                #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
                #Calculates distancePoseStamped
            current_distance= speed*(t1-t0)
            #After the loop, stops the robot
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
            #Force the robot to stop
        self.velocity_publisher.publish(vel_msg)
        return

    def rotate(self, cw, angle, absolute=False):
        turn_msg = Twist()
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
        if not absolute:
            current_angle = 0
        else:
            current_angle = self.pose.theta

        while(current_angle < relative_angle):
            self.velocity_publisher.publish(turn_msg)
            #self.rate.sleep()
            t1 = rospy.Time.now().to_sec()
            current_angle = angular_speed*(t1-t0)

        #Forcing our robot to stop
        turn_msg.angular.z = 0
        self.velocity_publisher.publish(turn_msg)
        #self.rate.sleep()
        #rospy.spin()
        return

    def stop(self):
        # send zero velocity to robot
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.sleep(1)
        print("Stopped")

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=0.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def move2goal(self, x, y):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = (x*0.7) + 5.5
        goal_pose.y = (y*0.7) + 5.5

        print("at " + str((self.pose.x-5.5)/0.7) + " " + str((self.pose.y-5.5)/0.7))
        #print(str(self.steering_angle(goal_pose)))
        print("moving to " + str(x) + "," + str(y))

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.01

        vel_msg = Twist()

        while self.euclidean_distance(goal_pose) >= distance_tolerance:

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control
            #print("in loop")
            #line = " ".join([str(self.pose.x), str(self.pose.y), str(self.euclidean_distance(goal_pose))])
            #print(line)
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
            #self.rate.sleep()

    def orient2goal(self, x, y, target_angle):
        self.move2goal(x, y)
        self.stop()
        self.rotate(True, target_angle, True)

    def circle(self, radius, speed=5, rotations=1):
        distance = radius * 2 * PI 
        time = distance/speed
        angular = 360/time
        angular_vel = angular*2*PI/360

        iteration = 0
        vel_msg = Twist()
        
        vel_msg.linear.x = abs(speed)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular_vel

        #while not rospy.is_shutdown():

        #Setting the current time for distance calculus
        t0 = rospy.Time.now().to_sec()
        current_angle = 0
        total_angle = 360*rotations
            #Loop to move the turtle in an specified distance
        while(current_angle < total_angle):
                #Publish the velocity
            self.velocity_publisher.publish(vel_msg)
                #Takes actual time to velocity calculus
            t1=rospy.Time.now().to_sec()
                #Calculates distancePoseStamped
            current_angle = angular*(t1-t0)
            #After the loop, stops the robot
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.angular.z = 0
            #Force the robot to stop
        self.velocity_publisher.publish(vel_msg)
        return






if __name__ == '__main__':
    try:
        x = TurtleBot()
        x.circle(2, rotations=5)

    except rospy.ROSInterruptException:
        pass

