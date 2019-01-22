#!/usr/bin/env python

#Import sys to be able to parse command line arguments and read waypoints
import sys

#Import rospy, message types and math for calculations
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, pi, acos

# Create a dictionary to store the x, y and rotation values for each waypoint as a dictionary
# Waypoints are numbered in the order they appear in the .txt file
waypoints = {}

with open(sys.argv[1]) as f:
    counter = 1
    for line in f:
        if line.strip():
            waypoint = line.split()
            waypoints[counter] = waypoint
            counter +=1

class Robot_controller:

    def __init__(self):
        # Creates a node with name 'Robot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('robot_control', anonymous=True)

        # Publisher which will publish a message of type 'Twist' to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/robot0/cmd_vel',
                                                  Twist, queue_size=10)
        
        # A subscriber to the topic '/turtle1/pose'. 
        # self.update_pose is called when a message of type 'Odometry' is received.
        self.pose_subscriber = rospy.Subscriber('/robot0/odom',
                                                Odometry, self.update_pose)

        # Initialize pose variable (of type 'Pose') to store our current pose
        self.pose = Pose()
        
        #Initialize variables to store the current x and y coordiantes and current orientation (theta)
        self.x = 0
        self.y = 2
        self.theta = 0

        # Set the distance and angle tolerance, decreased tolerance to increase accuracy
        self.distance_tolerance = 0.1
        self.angle_tolerance = 0.1

        self.rate = rospy.Rate(10)


    def update_pose(self,data):
        """Callback function which is called when a new message of type Odometry is
        received by the subscriber."""
        #Odomotry has a nested data structure. The pose is an attribute of pose with covariance 
        self.pose = data.pose.pose
        self.theta = 2 * atan2(self.pose.orientation.z, self.pose.orientation.w) * 180 / pi
        self.x = round(self.pose.position.x, 1)
        self.y = round(self.pose.position.y, 1) 

    def euclidean_distance(self,x,y):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((x - self.x), 2) +
                    pow((y - self.y), 2))
    #Works out the distance using pythagerous therom 

    def linear_vel(self,x,y, constant=1):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(x,y)
    #Multiplies distance by constant to increase speed of movement

    def steering_angle(self,x,y):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(y - self.y, x - self.x)
    #Using tan in a triangle works out the direction to move

    def angular_vel(self,x,y, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(x,y) - self.theta*pi/180)
    #Speed at rotation is done.

    def wp_angle(self,goal_x, goal_y):

        hyp = sqrt(pow(goal_x, 2) + pow(goal_y - self.y, 2))
        y = sqrt( pow(goal_x -self.x ,2) + pow(goal_y - self.y, 2))
    

        angle = acos((pow(self.x, 2) + pow(y,2) - pow(hyp, 2))/(2*self.x*y))
        
        return (pi - angle)


    def move2goal(self, waypoint):
        """Moves the robot to the goal."""


        # Set goal position and orientation to waypoint values
        goal_x = int(waypoint[0])
        goal_y = int(waypoint[1])
        goal_theta = int(waypoint[2])

        vel_msg = Twist()

        if goal_y - self.y > self.distance_tolerance:
            print('meh', goal_y, self.y)
            turning_angle = self.wp_angle(goal_x, goal_y)
        else:
            turning_angle = 0 

        while self.euclidean_distance(goal_x,goal_y) >= self.distance_tolerance:

            # if self.x == 0:

            #     while abs(self.steering_angle(goal_x, goal_y)-self.theta*pi/180) >= 0.1:
            #         vel_msg.angular.z = self.angular_vel(goal_x,goal_y,constant=3)
            #         self.velocity_publisher.publish(vel_msg)
            #         self.rate.sleep()
            #else:
                
            while abs(turning_angle - (self.theta*pi/180)) >= 0.1:
                print('wp_angle = {}'.format((self.wp_angle(goal_x, goal_y))*180/pi))
                print('self theta = {}'.format(self.theta))
            
                
                #print(self.steering_angle(goal_x, goal_y)-(self.theta*180)/pi)
                vel_msg.angular.z = (turning_angle - (self.theta*pi/180))*2

                print(vel_msg)

                self.velocity_publisher.publish(vel_msg)
                self.rate.sleep()

            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_x,goal_y)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

          
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_x,goal_y)
            vel_msg.angular.z = 0
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            # Publish at the desired rate.
            self.rate.sleep()

        

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        while abs(goal_theta-self.theta)  >= self.angle_tolerance:
     
            vel_msg.angular.z = ((goal_theta-self.theta)*pi/180)*2
            
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()
        
        # Stopping our robot after the movement is over.
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        rospy.loginfo('Current position, x: {}, y:{}, theta:{}'.format(self.x,
                self.y, self.theta))

    # If we press control + C, the node will stop.

if __name__ == '__main__':
    try:
        x = Robot_controller()
        #Works through set waypoints
        for waypoint in waypoints.values():
            print(waypoint)
            x.move2goal(waypoint)
            rospy.sleep(2)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

