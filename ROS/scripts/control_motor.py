#!/use/bin/evn python

import rospy
from std_msgs.msg import Int16
# from geometry_msgs  import Twist
# nasser =Twist
def talker():
    pub=rospy.Publisher('PWM_motor',Int16,queue_size=10)
    rospy.init_node('pwm_motor',anonymous=True)
    off=rospy.Rate(1)
    on=rospy.Rate(0.3)
    while not rospy.is_shutdown():
        rospy.loginfo(128)
        pub.publish(128)
        on.sleep()

        rospy.loginfo(255)
        pub.publish(255)
        on.sleep()

        rospy.loginfo(0)
        pub.publish(0)
        off.sleep()

        rospy.loginfo(-128)
        pub.publish(-128)
        on.sleep()

        rospy.loginfo(-255)
        pub.publish(-255)
        on.sleep()

        rospy.loginfo(0)
        pub.publish(0)
        off.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInternalException:
        pass