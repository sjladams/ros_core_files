from nodes import Nodes
import numpy as np
import time

# mapper_dict = {0: 1,
#                1: 2}

# callibration_dict = {0: 0.05,
#                      1: 0.042}



active_robots = ['3803', '3735']


nodes = Nodes(active_robots)

nodes.callibrate()
nodes.pendle_experiment()

# for tag in nodes.nodes:
#     # tag = 1
#     # print(nodes.nodes[tag].orien)
#     # start_pose = nodes.nodes[tag].pose
#     # start_time = nodes.nodes[tag].odomMsg_time
#     # nodes.nodes[tag].publish_greenLed(np.array([30]))
#     # nodes.nodes[tag].publish_motor_vlt(np.array([10, 10]))
#     # time.sleep(2)
#     # end_pose = nodes.nodes[tag].pose
#     # end_time = nodes.nodes[tag].odomMsg_time
#     # nodes.nodes[tag].publish_motor_vlt()
#     # nodes.nodes[tag].publish_greenLed()
#     # print(nodes.nodes[tag].orien)
#
#     # print(np.linalg.norm(end_pose-start_pose) / (end_time-start_time))
#
#     nodes.nodes[tag].callibrate()




print('en afgebroken')