import rospy
from std_msgs.msg import Int16MultiArray,Float32MultiArray
import numpy as np
import time
import pygame
from pygame.locals import *
import numpy as np

def my_callback(state):
    rospy.loginfo("\nPressure: %d mpar\tWater Temperature: %.2f °C\tDepth: %.2f m\nAccX: %d\tAccY: %d \tAccZ: %d \nGyX: %d\tGyY: %d \tGyY: %d \nInside Temperature: %d °C"\
                  ,state.data[0],state.data[1],state.data[2],state.data[3],state.data[4],state.data[5]\
                  ,state.data[6],state.data[7],state.data[8],state.data[9])



def listener():
    rospy.Subscriber('read_sensors',Float32MultiArray,my_callback)
    rospy.init_node('display_sensors',anonymous=True)

    rospy.spin()
        

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInternalException:
        pass