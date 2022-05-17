#! /usr/bin/env python3

import time
import rospy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Empty, EmptyRequest


class MoveHusky():

    def __init__(self):

        self.cmd = Twist()

        self.husky_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        while self.husky_vel_pub.get_num_connections() < 1:
            rospy.loginfo("Waiting for connection to /cmd_vel")
            time.sleep(0.1)

        self.sub_msg = PoseWithCovarianceStamped()
        self.amcl_sub = rospy.Subscriber(
            '/amcl_pose', PoseWithCovarianceStamped, self.amcl_callback)
        while self.amcl_sub.get_num_connections() < 1:
            rospy.loginfo("Waiting for connection to /amcl_pose")
            time.sleep(0.1)

        rospy.loginfo("Waiting for /global_localization service")
        rospy.wait_for_service('/global_localization')
        self.particles_srv = rospy.ServiceProxy('/global_localization', Empty)

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
        self.rate = rospy.Rate(10)

    def shutdownhook(self):
        # works better than the rospy.is_shut_down()
        self.stop_husky()
        self.ctrl_c = True

    def amcl_callback(self, msg):
        self.sub_msg = msg

    def move_forward(self, linear_speed=0.5, angular_speed=0.0):

        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        i = 0

        while i < 30:
            self.husky_vel_pub.publish(self.cmd)
            self.rate.sleep()
            i += 1

    def turn(self, linear_speed=0.0, angular_speed=0.7):

        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        i = 0

        while i < 25:
            self.husky_vel_pub.publish(self.cmd)
            self.rate.sleep()
            i += 1

    def stop_husky(self):

        rospy.loginfo("Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        i = 0

        while i < 20:
            self.husky_vel_pub.publish(self.cmd)
            self.rate.sleep()
            i += 1

    def move_square(self):

        i = 0

        while not self.ctrl_c and i < 4:
            # Move Forwards
            rospy.loginfo("######## Going Forwards...")
            self.move_forward()
            self.stop_husky()
            # Turn
            rospy.loginfo("######## Turning...")
            self.turn()
            self.stop_husky()
            i += 1

        self.stop_husky()
        rospy.loginfo("######## Finished Moving in a Square")

    def calculate_covariance(self):

        rospy.loginfo("######## Calculating Covariance...")
        cov_x = self.sub_msg.pose.covariance[0]
        cov_y = self.sub_msg.pose.covariance[7]
        cov_z = self.sub_msg.pose.covariance[35]
        rospy.loginfo("## Cov X: " + str(cov_x) + " ## Cov Y: " +
                      str(cov_y) + " ## Cov Z: " + str(cov_z))
        cov_res = (cov_x + cov_y + cov_z) / 3

        return cov_res

    def call_service(self):

        rospy.loginfo("######## Calling Service...")
        msg_for_service = EmptyRequest()
        result = self.particles_srv(msg_for_service)
        print(result)


if __name__ == '__main__':
    rospy.init_node('move_husky_node')
    MoveHusky_object = MoveHusky()

    cov = 1

    while cov > 0.65:
        MoveHusky_object.call_service()
        MoveHusky_object.move_square()
        cov = MoveHusky_object.calculate_covariance()
        rospy.loginfo("######## Total Covariance: " + str(cov))
        if cov > 0.65:
            rospy.loginfo(
                "######## Total Covariance is greater than 0.65. Repeating the process...")
        else:
            rospy.loginfo(
                "######## Total Covariance is lower than 0.65. Robot correctly localized!")
            rospy.loginfo("######## Exiting...")

    rospy.loginfo("### Finish ###")
