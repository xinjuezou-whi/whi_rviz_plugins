#!/usr/bin/env python

import roslib; roslib.load_manifest('whi_rviz_plugins')
from whi_interfaces.msg import WhiBattery
import rospy
from math import cos, sin
import tf

topic = 'test_bat'
publisher = rospy.Publisher(topic, WhiBattery, queue_size=5)

rospy.init_node(topic)

br = tf.TransformBroadcaster()
rate = rospy.Rate(10)
angle = 0

step = 5
while not rospy.is_shutdown():

    battery = WhiBattery()
    battery.header.frame_id = "/base_link"
    battery.header.stamp = rospy.Time.now()
   
    battery.percent = step;
    publisher.publish(battery)

    br.sendTransform((0, 0, 0),
                     tf.transformations.quaternion_from_euler(0, 0, angle),
                     rospy.Time.now(),
                     "base_link",
                     "map")
    step = (step + 5) % 100;
    rate.sleep()
