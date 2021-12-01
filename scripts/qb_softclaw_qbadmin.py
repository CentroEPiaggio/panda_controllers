#!/usr/bin/env python
# license removed for brevity

# PUT THIS SCRIPT IN QBADMIN/BIN_UNIX

import os
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

open_equilibrium = 0.4
close_equilibrium = 0.0
high_preset = 0.2
low_preset = 0.0

publisher_1 = rospy.Publisher('/soft_claw/reference_1', Float64MultiArray, queue_size=10)
publisher_2 = rospy.Publisher('/soft_claw/reference_2', Float64MultiArray, queue_size=10)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard command %s", data.data)

    command = data.data

    if command > 1.0 :
        command = 1.0
    if command < 0.0 :
        command = 0.0

    if command > 0.5 :
        os.system('./qbadmin 1 -s 0')
    elif command < 0.5 :
        os.system('./qbadmin 1 -s 5000')
    else :
        os.system('./qbadmin 1 -s 0')


def gripper_control():

    rospy.init_node('soft_claw_control', anonymous=True)

    rospy.Subscriber("/panda_controllers/gripper_control", Float64, callback)

    rate = rospy.Rate(10) # 10hz

    os.system('./qbadmin 1 -a')

    hello_str = "Starting to run soft_claw_control at time = %s" % rospy.get_time()
    rospy.loginfo(hello_str)

    while not rospy.is_shutdown():

        rate.sleep()


if __name__ == '__main__':
    try:
        gripper_control()
    except rospy.ROSInterruptException:
        pass
