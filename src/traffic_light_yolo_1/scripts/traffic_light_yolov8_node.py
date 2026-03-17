#!/usr/bin/env python3
# coding=utf-8

import rospy
from std_msgs.msg import String

def main():

    rospy.init_node("traffic_light_yolo_node")

    pub = rospy.Publisher("/traffic_light_state", String, queue_size=10)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        msg = "traffic light detection running"
        pub.publish(msg)

        rospy.loginfo(msg)

        rate.sleep()

if __name__ == "__main__":
    main()