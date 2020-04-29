#!/usr/bin/env python
import rospy
import os 
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt, tan

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
    
    def lateral_move(self, isForward, dist_x, speed=1):
        vel_msg = Twist()
        #Checking if the movement is forward or backwards
        if(isForward):
            if dist_x > 0:
                vel_msg.linear.x = abs(speed)
                distance = dist_x
        else:
            if dist_x > 0:
                vel_msg.linear.x = -abs(speed)
                distance = dist_x

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

    def orientation_vel(self, goal_pose, constant=6):
        return min(constant * (goal_pose - self.pose.theta), 1)

    def orient(self, goal_theta):
        angle_tolerance = 0.001
        #if goal_theta < 0:
        #    goal_theta = 360 + goal_theta
        goal_theta = goal_theta * 2 * PI/360
        print("goal: " + str(goal_theta))
        vel_msg = Twist()

        while abs(goal_theta - self.pose.theta) >= angle_tolerance:
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.orientation_vel(goal_theta)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    def move2goal(self, x, y, absolute):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        if not absolute:
            goal_pose.x = (x*0.7) + 5.5
            goal_pose.y = (y*0.7) + 5.5
        else:
            goal_pose.x = x
            goal_pose.y = y

        print("at " + str((self.pose.x-5.5)/0.7) + " " + str((self.pose.y-5.5)/0.7))
        #print(str(self.steering_angle(goal_pose)))
        print("moving to " + str(x) + "," + str(y))

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.001

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

    def orient2goal(self, start_pose, end_pose):
        x0, y0, theta0 = start_pose
        xf, yf, thetaf = end_pose

        command = "rosservice call /turtle1/teleport_absolute " + str(x0) + " " + str(y0) + " " + str(theta0) 
        os.system(command)
        print(xf, yf)
        self.move2goal(xf, yf, True)
        self.stop()
        self.orient(thetaf)
        print(self.pose)

    def circle(self, radius, speed=5, rotations=1):
        distance = radius * 2 * PI 
        time = distance/speed
        angular = 360/time
        angular_vel = angular*2*PI/360

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

    def spiral_angular_velocity(self):
        x = self.pose.dist_x
        y = self.pose.dist_y
        r = sqrt(pow(x, 2) + pow(y, 2))
        distance = r * 2 * PI 
        time = distance/speed
        angular = 360/time * 0.9
        angular_vel = angular*2*PI/360
        return angular_vel
    
    def draw_obstacle(self):
        self.orient(-90)
        self.lateral_move(True, 1)
        self.orient(-180)
        self.lateral_move(True, 1)
        self.orient(90)
        self.lateral_move(True, 1)
        self.orient(0)
        self.lateral_move(True, 1)
        return

    def spiral(self, radius, speed=10, rotations=1):
        distance = radius * 2 * PI 
        time = distance/speed
        angular = 360/time * 4
        angular_vel = angular*2*PI/360

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
        last_angle = 0
        iteration = 0
            #Loop to move the turtle in an specified distance
        while(iteration < rotations):
                #Publish the velocity
            t1=rospy.Time.now().to_sec()
            #vel_msg.angular.z = self.spiral_angular_velocity()
            vel_msg.linear.x = speed * (t1-t0)
            self.velocity_publisher.publish(vel_msg)
            current_angle = angular*(t1-t0)
            if self.pose.theta > 0 and last_angle < 0:
                iteration += 1
            last_angle = self.pose.theta
            print(str(current_angle))
            print(self.pose.theta)
            print(self.pose.theta * 360 / (2*PI))
            print(self.pose.x, self.pose.y)
            print(iteration)
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
        #x.orient2goal(10, 0, 0, start_angle=None, absolute=True)
        ##Part1, example 1
        #x.orient2goal([0, 0, 45], [10, 0, 0])

        ##Part2, example 2
        #x.orient2goal([0, 0, 90], [8, 0, -90])

        ##Part 2a
        #x.spiral(3, rotations=7)

        ##Part 2b
        x.draw_obstacle()
        x.spiral(6.5, rotations=3)

    except rospy.ROSInterruptException:
        pass

