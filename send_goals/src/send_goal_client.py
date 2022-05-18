#! /usr/bin/env python
import rospy
import time
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback


def feedback_callback(feedback):

    print('[Feedback] Going to Goal Pose...')


rospy.init_node('move_base_action_client')

client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
client.wait_for_server()


goals = []

goals.append(MoveBaseGoal())
goals[0].target_pose.header.frame_id = 'map'
goals[0].target_pose.pose.position.x = 1.6
goals[0].target_pose.pose.position.y = -2.9
goals[0].target_pose.pose.position.z = 0.0
goals[0].target_pose.pose.orientation.x = 0.0
goals[0].target_pose.pose.orientation.y = 0.0
goals[0].target_pose.pose.orientation.z = -0.5
goals[0].target_pose.pose.orientation.w = 0.9

goals.append(MoveBaseGoal())
goals[1].target_pose.header.frame_id = 'map'
goals[1].target_pose.pose.position.x = 1.8
goals[1].target_pose.pose.position.y = 1.7
goals[1].target_pose.pose.position.z = 0.0
goals[1].target_pose.pose.orientation.x = 0.0
goals[1].target_pose.pose.orientation.y = 0.0
goals[1].target_pose.pose.orientation.z = 0.9
goals[1].target_pose.pose.orientation.w = 0.4

goals.append(MoveBaseGoal())
goals[2].target_pose.header.frame_id = 'map'
goals[2].target_pose.pose.position.x = -2.5
goals[2].target_pose.pose.position.y = -1.2
goals[2].target_pose.pose.position.z = 0.0
goals[2].target_pose.pose.orientation.x = 0.0
goals[2].target_pose.pose.orientation.y = 0.0
goals[2].target_pose.pose.orientation.z = -0.6
goals[2].target_pose.pose.orientation.w = 0.8

print(goals)

client.send_goal(goals[0], feedback_cb=feedback_callback)

# Uncomment these lines to test goal preemption:
# time.sleep(3.0)
# client.cancel_goal()  # would cancel the goal 3 seconds after starting


client.wait_for_result()

print("[Result] State: ", (client.get_state()))
