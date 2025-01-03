import rospy
from std_msgs.msg import Float32

def my_callback(dist):
    rospy.loginfo("The Dist is:  %f" , dist.data)
    rate=rospy.Rate(5)
    rate.sleep()

def listener():
    rospy.Subscriber('chatter',Float32,my_callback)
    rospy.init_node('listener',anonymous=True)

    rospy.spin()
        

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInternalException:
        pass