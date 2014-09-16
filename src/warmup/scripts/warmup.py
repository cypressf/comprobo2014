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
DANGER_ZONE_LENGTH = 1.0
DANGER_ZONE_WIDTH = 0.5


def degrees(radians):
    radians %= TAU
    degree_index = math.degrees(radians)
    return int(degree_index)


def ranges_to_points(ranges):
    points = []
    for angle, radius in enumerate(ranges):
        points.append(Point(length=radius, angle=angle))
    return points


def is_in_front_left(point):
    return 0 < point.angle_radians <= TAU * 1/4.0


def is_in_back_left(point):
    return TAU * 1/4.0 < point.angle_radians <= TAU * 1/2.0


def is_in_back_right(point):
    return TAU * 1/2.0 < point.angle_radians <= TAU * 3/4.0


def is_in_front_right(point):
    return TAU * 3/4.0 < point.angle_radians <= TAU


def is_in_danger_zone(point):
    a = DANGER_ZONE_LENGTH * math.sin(point.angle_radians)
    b = DANGER_ZONE_WIDTH * math.cos(point.angle_radians)
    max_radius = (DANGER_ZONE_LENGTH * DANGER_ZONE_WIDTH) / math.sqrt(a**2 + b**2)
    return point.length < max_radius


class Point:

    def __init__(self, length=0.0, angle=0.0):
        self.length = length # meters
        self.angle_degrees = angle
        self.angle_radians = math.radians(angle)

    def __lt__(self, other):
        return self.length < other.radius

    def __str__(self):
        return "radius: %.2f  angle: %.3f" % (self.length, self.angle_radians / TAU)


class Controller:

    def __init__(self):
        self.front_points = []
        self.proportion_constant = 1.0
        self.goal = 1.0  # meters
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = None
        self.move = self.turn_right
        self.running = False
        signal.signal(signal.SIGINT, self.signal_handler)
        rospy.init_node('controller', anonymous=True)

    def is_in_danger(self):
        return len(self.get_danger_points()) > 0

    def get_danger_points(self):
        return [point for point in self.front_points if is_in_danger_zone(point)]

    def find_path(self):
        danger_points = self.get_danger_points()
        left_danger_points = [point for point in danger_points if is_in_front_left(point)]
        right_danger_points = [point for point in danger_points if is_in_front_right(point)]

        if not danger_points:
            self.move = self.move_forward
            return self.move_forward()

        if len(left_danger_points) < len(right_danger_points):
            self.move = self.turn_left
            return self.turn_left()
        else:
            self.move = self.turn_right
            return self.turn_right()

    def turn_left(self):
        if not self.is_in_danger():
            self.move = self.move_forward
            return self.move_forward()

        return Twist(angular=Vector3(z=MAX_ANGULAR_SPEED))

    def turn_right(self):
        if not self.is_in_danger():
            self.move = self.move_forward
            return self.move_forward()
        rospy.loginfo(len(self.get_danger_points()))

        return Twist(angular=Vector3(z=-MAX_ANGULAR_SPEED))

    def move_forward(self):
        if self.is_in_danger():
            self.move = self.find_path

        return Twist(linear=Vector3(x=MAX_LINEAR_SPEED))

    def scan_received(self, laser_scan_msg):
        """
        Process new LaserScan message.
        """
        points = ranges_to_points(laser_scan_msg.ranges)

        front_points = [point for point in points if is_in_front_left(point) or is_in_front_right(point)]
        self.front_points = [point for point in front_points if 0 < point.length < 5]
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