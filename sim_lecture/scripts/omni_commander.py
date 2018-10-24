#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from math import *

twist_last = Twist()
twist_enable = False
jointstate = TFMessage()
def twist_stamped_callback(twist_msg):
    global twist_last
    global twist_enable
    twist_last = twist_msg
    twist_enable = True


wheel_base=sqrt(2*pow(0.2,2))
wheel_radius=0.019
wheel=[pi/4, 3*pi/4, 5*pi/4, 7*pi/4]
wheel_normal=[0,0,0,0]

def calculation(out, input):
    for i in range(4):
        out[i]=(cos(wheel_normal[i])*input[0]+sin(wheel_normal[i])*input[1])/wheel_radius
        out[i]+=input[2]*wheel_base/wheel_radius
    return out[0], out[1], out[2], out[3]


if __name__ == '__main__':
    rospy.init_node("omni_driver")
    #rosparam
    rospy.get_param("~wheel_base", wheel_base)
    rospy.get_param("~wheel_radius",wheel_radius)
    rospy.get_param("~wheel0", wheel[0])
    rospy.get_param("~wheel1", wheel[1])
    rospy.get_param("~wheel2", wheel[2])
    rospy.get_param("~wheel3", wheel[3])

    #publish
    wheel0_pub = rospy.Publisher("/odm_robot/joint_controller0/command",Float64,queue_size=10)
    wheel1_pub = rospy.Publisher("/odm_robot/joint_controller1/command",Float64,queue_size=10)
    wheel2_pub = rospy.Publisher("/odm_robot/joint_controller2/command",Float64,queue_size=10)
    wheel3_pub = rospy.Publisher("/odm_robot/joint_controller3/command",Float64,queue_size=10)

    #Subscribe
    rospy.Subscriber("cmd_vel",Twist,twist_stamped_callback)
    #rospy.Subscriber("/tf",TFMessage,tf_callback)
    for i in range(4):
        wheel_normal[i]=wheel[i]-pi/2

    loop_rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if twist_enable:
            input=[0,0,0]
            out=[0,0,0,0]
            input[0]=twist_last.linear.x
            input[1]=twist_last.linear.y
            input[2]=twist_last.angular.z
            out[0], out[1], out[2], out[3] = calculation(out,input);
            data=[0,0,0,0];
            data[0]=Float64(out[0])
            data[1]=Float64(out[1])
            data[2]=Float64(out[2])
            data[3]=Float64(out[3])
            wheel0_pub.publish(data[0])
            wheel1_pub.publish(data[1])
            wheel2_pub.publish(data[2])
            wheel3_pub.publish(data[3])

        loop_rate.sleep()
