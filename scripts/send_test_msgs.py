#!/usr/bin/env python

import roslib; roslib.load_manifest('whi_rviz_plugins')
from whi_interfaces.msg import WhiBattery
import rospy
from math import cos, sin
import tf
import sys

topic = 'test_bat'
publisher = rospy.Publisher(topic, WhiBattery, queue_size=5)

rospy.init_node(topic)

br = tf.TransformBroadcaster()
rate = rospy.Rate(10)
roll = 0
pitch = 0
yaw = 0
withTransform = True if len(sys.argv) > 0 and sys.argv[0] == 'transform' else False

step = 5
while not rospy.is_shutdown():

  battery = WhiBattery()
  battery.header.frame_id = "battery"
  battery.header.stamp = rospy.Time.now()
   
  battery.soc = step;
  battery.state = WhiBattery.STA_NEED_CHARGING if step < 30 else WhiBattery.STA_NORMAL
  publisher.publish(battery)

  if withTransform:
    br.sendTransform((0, 0, 0),
      tf.transformations.quaternion_from_euler(roll, pitch, yaw),
      rospy.Time.now(),
      "battery",
      "map")

  step = (step + 5) % 100;
  rate.sleep()
