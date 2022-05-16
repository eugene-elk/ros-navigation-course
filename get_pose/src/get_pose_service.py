#! /usr/bin/env python3

import time
import rospy

from get_pose.srv import PositionMsg, PositionMsgResponse
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose


class GetPoseSrv():

    def __init__(self):
        self.robotPose = Pose()

        self.subScan = rospy.Subscriber(
            '/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        while self.subScan.get_num_connections() < 1:
            rospy.loginfo("Waiting for subscription to /amcl_pose")
            time.sleep(0.1)

        self.srv = rospy.Service(
            '/get_pose', PositionMsg, self.service_callback)
        self.rate = rospy.Rate(10)

    def amcl_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        self.robotPose = msg

    def service_callback(self, request):
        print("Robot Position: " + str(self.robotPose))
        result = PositionMsgResponse()
        result.x = self.x_pos
        result.y = self.y_pos
        return result


if __name__ == '__main__':
    rospy.init_node('get_pose_node')
    GetPoseSrv()
    rospy.spin()
