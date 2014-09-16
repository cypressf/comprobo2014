#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
import signal
import sys
import math

TAU = 2 * math.pi
MAX_LINEAR_SPEED = 0.2
MAX_ANGULAR_SPEED = 1.0

def degrees(radians):
    radians %= TAU
    degree_index = math.degrees(radians)
    if degree_index < 0:
        degree_index = 360 - degree_index
    return int(degree_index)


def translate_ranges(ranges):
    return ranges[269::] + ranges[0::269]


class Controller:

    def __init__(self):
        self.proportion_constant = 1.0
        self.goal = 1.0  # meters
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = None
        self.mean_distance = None
        self.closest_object = None
        self.move = self.move_forward
        self.running = False
        signal.signal(signal.SIGINT, self.signal_handler)
        rospy.init_node('controller', anonymous=True)

    def find_path(self):
        # if self.mean_distance > 1.0:
        #     self.move = self.move_forward

        if self.closest_object > 1.0:
            self.move = self.move_forward

        return Twist(angular=Vector3(z=MAX_ANGULAR_SPEED))

    def move_forward(self):
        # if self.mean_distance < 1.0:
        #     self.move = self.find_path

        if self.closest_object <= 1.0:
            self.move = self.find_path

        return Twist(linear=Vector3(x=MAX_LINEAR_SPEED))

    def approach_one_meter(self):
        if self.mean_distance is not None:
            error = self.mean_distance - self.goal
            velocity = Vector3(x=error * self.proportion_constant)
            message = Twist(linear=velocity)
            self.pub.publish(message)

    def scan_received(self, laser_scan_msg):
        """
        Process new LaserScan message.
        """
        ranges = translate_ranges(laser_scan_msg.ranges)
        begin_index = degrees(TAU * 1/8)
        end_index = degrees(TAU * 3/8)
        front_readings = ranges[begin_index:end_index]

        valid_front_readings = [reading for reading in front_readings if 0 < reading < 8]

        self.closest_object = min(valid_front_readings)

        if len(valid_front_readings) > 0:
            self.mean_distance = sum(valid_front_readings) / float(len(valid_front_readings))
            rospy.loginfo("Mean distance %f", self.mean_distance)

    def run(self):
        self.running = True
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_received)
        while not rospy.is_shutdown() and self.running:
            message = self.move()
            self.pub.publish(message)

    def signal_handler(self, signal, frame):
        self.running = False
        self.pub.publish(Twist())
        sys.exit(0)


if __name__ == '__main__':
    try:
        teleop = Controller()
        teleop.run()
    except rospy.ROSInterruptException:
        pass