import rospy
from std_msgs.msg import Int32

def my_callback(speed):
    rospy.loginfo("The Motor Speed: {} RPM".format(speed.data))

def listener():
    rospy.Subscriber('chatter',Int32,my_callback)
    rospy.init_node('listener',anonymous=True)

    rospy.spin()
        

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInternalException:
        pass