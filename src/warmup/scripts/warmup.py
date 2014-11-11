#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
import signal
import sys
import math
import tf

TAU = 2 * math.pi
MAX_LINEAR_SPEED = 0.3
MAX_ANGULAR_SPEED = 1.0
DANGER_ZONE_LENGTH = 1.0
DANGER_ZONE_WIDTH = 0.5
DANGER_POINTS_MULTIPLIER = 1/50.0
WALL_FOLLOW_MEASUREMENT_WIDTH = 1/32.0 * TAU
WALL_FOLLOW_DISTANCE = 1.0

def degrees(radians):
    radians %= TAU
    degree_index = math.degrees(radians)
    return int(degree_index)


def ranges_to_points(ranges):
    points = []
    for angle, length in enumerate(ranges):
        if length:
            points.append(Point(length=length, angle=angle))
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
        rospy.init_node('controller', anonymous=True)
        self.points = []
        self.front_points = []
        self.proportion_constant = 1.0
        self.goal = 1.0  # meters
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.listener = tf.TransformListener()
        self.sub = None
        self.move = self.manual_override
        self.manual_linear_velocity = Vector3()
        self.manual_angular_velocity = Vector3()
        self.running = False
        signal.signal(signal.SIGINT, self.signal_handler)

    def is_in_danger(self):
        return len(self.get_danger_points()) > 0

    def get_right_points(self):
        return [point for point in self.points if (5/8.0 * TAU) < point.angle_radians < (7/8.0 * TAU) and point.length > 0]

    def get_danger_points(self):
        return [point for point in self.front_points if is_in_danger_zone(point)]

    def get_wall_follow_front(self):
        return [point for point in self.points if (7/8.0 * TAU - WALL_FOLLOW_MEASUREMENT_WIDTH) < point.angle_radians < (7/8.0 * TAU + WALL_FOLLOW_MEASUREMENT_WIDTH)]

    def get_wall_follow_back(self):
        return [point for point in self.points if (5/8.0 * TAU - WALL_FOLLOW_MEASUREMENT_WIDTH) < point.angle_radians < (5/8.0 * TAU + WALL_FOLLOW_MEASUREMENT_WIDTH)]

    def follow_wall(self):
        if self.is_in_danger():
            self.move = self.find_path
            return self.find_path()

        front_points = self.get_wall_follow_front()
        back_points = self.get_wall_follow_back()
        if not back_points or not front_points:
            self.move = self.move_forward
            return self.move_forward()

        front_points_total = 0

        for point in front_points:
            front_points_total += point.length

        front_points_total /= float(len(front_points))

        back_points_total = 0
        for point in back_points:
            back_points_total += point.length

        back_points_total /= float(len(back_points))


        points_diff = (back_points_total - front_points_total) / (back_points_total + front_points_total)

        linear_velocity = Vector3(x=MAX_LINEAR_SPEED)
        rospy.loginfo(linear_velocity)
        angular_velocity = Vector3(z=points_diff)

        return Twist(linear=linear_velocity, angular=angular_velocity)

    def find_path(self):
        danger_points = self.get_danger_points()
        left_danger_points = [point for point in danger_points if is_in_front_left(point)]
        right_danger_points = [point for point in danger_points if is_in_front_right(point)]

        if not danger_points:
            self.move = self.move_forward
            return self.move_forward()

        if len(left_danger_points) < len(right_danger_points):
            return self.turn_left(len(right_danger_points))
        else:
            return self.turn_right(len(left_danger_points))

    def turn_left(self, num_danger_points):
        rospy.loginfo("turn left")
        angular_velocity = Vector3(z=num_danger_points * DANGER_POINTS_MULTIPLIER * MAX_ANGULAR_SPEED)
        linear_velocity = Vector3(x=MAX_LINEAR_SPEED * (1 - num_danger_points * DANGER_POINTS_MULTIPLIER))
        return Twist(angular=angular_velocity, linear=linear_velocity)

    def turn_right(self, num_danger_points):
        rospy.loginfo("turn right")
        angular_velocity = Vector3(z=num_danger_points * DANGER_POINTS_MULTIPLIER * -MAX_ANGULAR_SPEED)
        linear_velocity = Vector3(x=MAX_LINEAR_SPEED * (1 - num_danger_points * DANGER_POINTS_MULTIPLIER))
        return Twist(angular=angular_velocity, linear=linear_velocity)

    def move_forward(self):
        if self.is_in_danger():
            self.move = self.find_path
            return self.find_path()

        front_points = self.get_wall_follow_front()
        back_points = self.get_wall_follow_back()
        if front_points and back_points:
            right_points = front_points + back_points
            right_points_avg = 0
            for point in right_points:
                right_points_avg += point.length
            right_points_avg /= float(len(right_points))

            if right_points_avg < WALL_FOLLOW_DISTANCE:
                self.move = self.follow_wall
                return self.follow_wall()

        rospy.loginfo("move forward")
        return Twist(linear=Vector3(x=MAX_LINEAR_SPEED))

    def scan_received(self, laser_scan_msg):
        """
        Process new LaserScan message.
        """
        self.points = ranges_to_points(laser_scan_msg.ranges)

        front_points = [point for point in self.points if is_in_front_left(point) or is_in_front_right(point)]
        self.front_points = [point for point in front_points if 0 < point.length]

    def manual_override(self):
        return Twist(linear=self.manual_linear_velocity, angular=self.manual_angular_velocity)

    def gamepad_callback(self, data):
        x_button = data.buttons[1]
        square_button = data.buttons[0]
        if x_button:
            self.move = self.manual_override
        if square_button:
            self.move = self.move_forward
        angular_value = data.axes[0]
        linear_value = data.axes[1]
        self.manual_linear_velocity = Vector3(x=linear_value * MAX_LINEAR_SPEED)
        self.manual_angular_velocity = Vector3(z=angular_value * MAX_ANGULAR_SPEED)

    def run(self):
        self.running = True
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_received)
        rospy.Subscriber("joy", Joy, self.gamepad_callback)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and self.running:
            # try:
            #     (trans, rot) = self.listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
            #     rospy.loginfo(trans)
            # except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #     pass
            movement = self.move()
            rospy.loginfo(movement)
            self.pub.publish(movement)
            rate.sleep()

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