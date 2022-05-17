#! /usr/bin/env python3

import rospy
from std_srvs.srv import Empty, EmptyRequest


rospy.init_node('service_client')
print("waiting for /global_localization service")
rospy.wait_for_service('/global_localization')
print("waiting finished")
disperse_particles_service = rospy.ServiceProxy('/global_localization', Empty)
msg = EmptyRequest()
result = disperse_particles_service(msg)
print(result)
