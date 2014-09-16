#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import signal
import sys
import math

TAU = 2 * math.pi
MAX_LINEAR_SPEED = 0.2
MAX_ANGULAR_SPEED = 0.2


def degrees(radians):
    radians %= TAU
    degree_index = math.degrees(radians)
    if degree_index < 0:
        degree_index = 360 - degree_index
    return int(degree_index)


def ranges_to_points(ranges):
    points = []
    for angle, radius in enumerate(ranges[269::] + ranges[0::269]):
        points.append(Point(radius=radius, angle=angle))
    return points


class Point:

    def __init__(self, radius=0.0, angle=0.0):
        self.radius = radius # meters
        self.angle_degrees = angle
        self.angle_radians = math.radians(angle)

    def __lt__(self, other):
        return self.radius < other.radius

    def __str__(self):
        return "radius: %.2f  angle: %.3f" % (self.radius, self.angle_radians / TAU)

class Controller:

    def __init__(self):
        self.points = []
        self.proportion_constant = 1.0
        self.goal = 1.0  # meters
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = None
        self.closest_point = Point(angle=0.0, radius=100.0)
        self.move = self.move_forward
        self.running = False
        signal.signal(signal.SIGINT, self.signal_handler)
        rospy.init_node('controller', anonymous=True)

    def find_path(self):
        # if self.mean_distance > 1.0:
        #     self.move = self.move_forward

        if self.closest_point.angle_radians <= TAU * 1/4.0:
            self.move = self.turn_left
            return self.turn_left()
        else:
            self.move = self.turn_right
            return self.turn_right()

    def turn_left(self):
        if self.closest_point.radius > 1.0:
            self.move = self.move_forward
            return self.move_forward()

        return Twist(angular=Vector3(z=MAX_ANGULAR_SPEED))

    def turn_right(self):
        if self.closest_point.radius > 1.0:
            self.move = self.move_forward
            return self.move_forward()

        return Twist(angular=Vector3(z=-MAX_ANGULAR_SPEED))

    def move_forward(self):
        # if self.mean_distance < 1.0:
        #     self.move = self.find_path

        if self.closest_point.radius <= 1.0:
            self.move = self.find_path

        return Twist(linear=Vector3(x=MAX_LINEAR_SPEED))

    def scan_received(self, laser_scan_msg):
        """
        Process new LaserScan message.
        """
        points = ranges_to_points(laser_scan_msg.ranges)
        front_points = [point for point in points if TAU * 1/8.0 < point.angle_radians < TAU * 3/8.0]
        valid_front_points = [point for point in front_points if 0 < point.radius < 8]

        if valid_front_points:
            self.closest_point = min(valid_front_points)

        rospy.loginfo(self.closest_point)
        self.pub.publish(self.move())

    def run(self):
        self.running = True
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_received)
        while not rospy.is_shutdown() and self.running:
            rospy.spin()

    def signal_handler(self, signal, frame):
        self.running = False
        self.pub.publish(Twist())
        sys.exit(0)


if __name__ == '__main__':
    try:
        controller = Controller()
        controller.run()
    except rospy.ROSInterruptException:
        pass