#!/usr/bin/env python

import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
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
        self.theta = Odometry.pose.pose.orientation()
        self.rate = rospy.Rate(10)
        

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data.pose.pose
        self.theta = 2 * atan2(self.pose.orientation.z, self.pose.orientation.w) * 180 / pi
        self.pose.position.x = round(self.pose.postion.x, 4)
        self.pose.position.y = round(self.pose.position.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.pose.postion.x), 2) +
                    pow((goal_pose.y - self.pose.postion.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.postion.y, goal_pose.x - self.pose.postion.x)

    def angular_vel(self, goal_pose, constant=6):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.theta)

    def move2goal(self, waypoint):
        """Moves the turtle to the goal."""
        goal_pose = Pose()

        # Get the input from the user.
        goal_pose.x = waypoint[0]
        goal_pose.y = waypoint[1]

        distance_tolerance = 0.1
        angle_tolerance = 5

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

        rospy.loginfo('Current position, x: {}, y:{}, theta:{}'.format(self.pose.position.x,
                self.pose.position.y, self.theta))

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

        # If we press control + C, the node will stop.
        rospy.spin()

        


if __name__ == '__main__':
    try:
        x = TurtleBot()
        for waypoint in waypoints:
            x.move2goal(waypoint)
    except rospy.ROSInterruptException:
        pass

