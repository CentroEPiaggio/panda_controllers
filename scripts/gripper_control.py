#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
from franka_gripper.msg import MoveActionGoal

def_speed = 0.05
open_width = 0.08
publisher = rospy.Publisher('/franka_gripper/move/goal', MoveActionGoal, queue_size=10)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard command %s", data.data)

    command = data.data
    if command > 1.0 :
        command = 1.0
    if command < 0.0 :
        command = 0.0

    control_msg = MoveActionGoal()
    control_msg.goal.width = (1 - command)*open_width
    control_msg.goal.speed = def_speed
    publisher.publish(control_msg)

def gripper_control():

    rospy.init_node('gripper_control', anonymous=True)

    rospy.Subscriber("/panda_controllers/gripper_control", Float64, callback)

    rate = rospy.Rate(10) # 10hz

    hello_str = "Starting to run gripper_control at time = %s" % rospy.get_time()
    rospy.loginfo(hello_str)

    while not rospy.is_shutdown():
        
        rate.sleep()


if __name__ == '__main__':
    try:
        gripper_control()
    except rospy.ROSInterruptException:
        pass