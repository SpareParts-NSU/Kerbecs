"""kerbecs_controller controller."""

import rospy
from std_msgs.msg import String
from kerbecs import *

robot = Kerbecs()

pub = rospy.Publisher('chatter', String, queue_size=10)
rospy.init_node('talker', anonymous=True)
# rate = rospy.Rate(10) # 10hz

while not rospy.is_shutdown():
    robot.stand()
    hello_str = "wud up my dudes %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    robot.crouch()
    hello_str = "wud up my dudes %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    pub.publish(hello_str)
    # rate.sleep()

# robot.stand()
# robot.crouch()
# robot.walk()