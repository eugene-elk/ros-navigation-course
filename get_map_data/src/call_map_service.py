#! /usr/bin/env python

import rospy
from nav_msgs.srv import GetMap, GetMapRequest


rospy.init_node('call_map_service_client')
print("Waiting for service server")
rospy.wait_for_service('/static_map')

print("Send request to service server")
map_service = rospy.ServiceProxy('/static_map', GetMap)
map_request = GetMapRequest()
result = map_service(map_request)
print(result)
