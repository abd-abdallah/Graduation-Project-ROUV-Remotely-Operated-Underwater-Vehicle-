import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8

def my_callback(data):
    rospy.loginfo(data.axes[1])

def listener():
    rospy.Subscriber('joy',Joy,my_callback)
    rospy.init_node('listener',anonymous=True)

    rospy.spin()
        

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInternalException:
        pass

