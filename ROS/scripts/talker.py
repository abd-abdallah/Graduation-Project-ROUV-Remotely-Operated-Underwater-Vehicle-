#!/use/bin/evn python

import rospy
from std_msgs.msg import String

def talker():
    i=0
    pub=rospy.Publisher('chatter',String,queue_size=10)
    rospy.init_node('talker',anonymous=True)
    rate=rospy.Rate(1)
    while not rospy.is_shutdown():
        i+=1
        hello_str = "Hello World {}".format(i) 
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    # global freq
    # freq=rospy.get_param("freq")
    try:
        talker()
    except rospy.ROSInternalException:
        pass