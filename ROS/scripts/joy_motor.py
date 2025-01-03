#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray

def PWM_callback(state):

    
    pub = rospy.Publisher("PWM_motor",Int16MultiArray, queue_size=1)


    #joy stick 
    joyOne_X=int(state.axes[0]*255)
    joyOne_Y=int(state.axes[1]*255)
    joyTwo_X=int(state.axes[3]*255)
    joyTwo_Y=int(state.axes[4]*-400+1500)


    #  axes buttons
    X_dir=int(state.axes[6]*255)
    Y_dir=int(state.axes[7])

    # buttons
    Right_B=state.buttons[1]
    Left_B= state.buttons[3]
    Upper_B=state.buttons[2]
    Lower_B=state.buttons[0]
    R1_pause=state.buttons[5]


    if((joyOne_Y >180) or (joyOne_Y < -180)):
        joyOne_X=0
    if((joyOne_X >180) or (joyOne_X < -180)):
        joyOne_Y=0


    # linear motion of ROV in Z dirction:
    T200Motors=joyTwo_Y

    # linear motion of ROV in  Y and X dirction and rotat about  Z dirction:
    JohnsonM1=-joyOne_Y+joyOne_X+X_dir
    JohnsonM2=-joyOne_Y-joyOne_X-X_dir
    JohnsonM3=joyOne_Y+joyOne_X-X_dir
    JohnsonM4=joyOne_Y-joyOne_X+X_dir


    # # Array carry The Situations of Joy stick
    motion_fo_motors=[JohnsonM1,JohnsonM2,JohnsonM3,JohnsonM4, T200Motors]
    pwm_motor=Int16MultiArray(data=motion_fo_motors)

    rospy.loginfo("\nJohnM1: %d   JohnM2: %d   JohnM3: %d   JohnM4: %d\nT200M5: %d   T200M6: %d"\
                  ,(joyOne_Y-joyOne_X),(joyOne_Y+joyOne_X),(joyOne_Y+joyOne_X),(joyOne_Y-joyOne_X),joyTwo_Y,joyTwo_Y)

    pub.publish(pwm_motor)



def joy_listener():

    # start node
    rospy.init_node("joy_motor", anonymous=True)
    rospy.Subscriber("joy", Joy, PWM_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        joy_listener()
    except rospy.ROSInterruptException:
        pass
