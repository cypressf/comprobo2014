#!/usr/bin/env python

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

class Teleop:

    def __init__(self):
        self.proportion_constant = 1.0
        self.goal = 1.0 # meters
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.mean_distance = 0.0
        rospy.init_node('teleop', anonymous=True)

    def scan_received(self, laser_scan_msg):
        """
        Process new LaserScan message.
        """
        valid_ranges = []
        for i in range(5):
            if msg.ranges[i] > 0 and msg.ranges[i] < 8:
                valid_ranges.append(msg.ranges[i])
        if len(valid_ranges) > 0:
            self.mean_distance = sum(valid_ranges) / float(len(valid_ranges))
            rospy.loginfo("Mean distance %f", self.mean_distance)

    def run(self):
        self.sub = rospy.Subscriber('scan', LaserScan, self.scan_received)
        while not rospy.is_shutdown():
            error = self.mean_distance - self.goal
            velocity = Vector3(x = error * self.proportion_constant)
            msg = Twist(linear = velocity)
            self.pub.publish(msg)
        self.pub.publish(Twist())

        
if __name__ == '__main__':
    try:
        teleop = Teleop()
        teleop.run()
    except rospy.ROSInterruptException: pass