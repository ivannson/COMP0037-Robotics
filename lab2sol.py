#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

from math import pow, atan2, sqrt, pi

waypoints = {}

with open(sys.argv[1]) as f:
    counter = 1
    for line in f:
        if line.strip():
            waypoint = line.split()
            waypoints[counter] = waypoint
            counter +=1

class Robot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('robot_control', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/robot0/cmd_vel',
                                                  Twist, queue_size=10)
        
        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber('/robot0/odom',
                                                Odometry, self.update_pose)

        self.pose = Pose()
        
        self.theta = 0
        self.x = 0
        self.y = 0
        self.rate = rospy.Rate(10)


    def update_pose(self,data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data.pose.pose

        self.theta = 2 * atan2(self.pose.orientation.z, self.pose.orientation.w) * 180 / pi
        self.x = round(self.pose.position.x, 4)
        self.y = round(self.pose.position.y, 4)
        print(self.x,self.y)

    def euclidean_distance(self,x,y):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((x - self.x), 2) +
                    pow((y - self.y), 2))

    def linear_vel(self,x,y, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(x,y)

    def steering_angle(self,x,y):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(y - self.y, x - self.x)

    def angular_vel(self,x,y, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(x,y) - self.theta)

    def move2goal(self, waypoint):
        """Moves the turtle to the goal."""

        # Get the input from the user.
        x = int(waypoint[0])
        y = int(waypoint[1])

        distance_tolerance = 0.1
        angle_tolerance = 5

        vel_msg = Twist()

        while self.euclidean_distance(x,y) >= distance_tolerance:
            print(self.euclidean_distance(x,y))

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(x,y)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = 0
            #vel_msg.angular.z = self.angular_vel(x,y)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

            # Publish at the desired rate.
            self.rate.sleep()

        rospy.loginfo('Current position, x: {}, y:{}, theta:{}'.format(self.pose.position.x,
                self.pose.position.y, self.theta))

        rospy.spin()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

    # If we press control + C, the node will stop.
   
if __name__ == '__main__':
    try:
        x = Robot()
        # for waypoint in waypoints.values():
        #     x.move2goal(waypoint)
        #     rospy.spin()
        #     rospy.sleep(1)
        waypoint_1 = waypoints[1]
        x.move2goal(waypoint)
    except rospy.ROSInterruptException:
        pass

