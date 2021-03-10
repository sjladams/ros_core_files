from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry

import rospy
import numpy as np
from functions import *
import time

mapper_dict = {'3803': {'std_speed': 0.05,
                        'optitrack_id': 1,
                        'elisa_id': 0
                        },
               '3735': {'std_speed': 0.05,
                        'optitrack_id': 2,
                        'elisa_id': 1
                        },
               }

wheel_distance = 0.041

class Node:
    def __init__(self, optitrack_id, elisa_id):
        self.optitrack_id = optitrack_id
        self.elisa_id = elisa_id

        self.pose = np.array([np.NaN, np.NaN])
        self.orien = np.NaN

        self.listener_optitrack_pose = rospy.Subscriber("/rigid_body_"+str(self.optitrack_id)+
                            "/pose", PoseStamped, self.listen_optitrack_pose_callback)
        self.listener_robot_pose = rospy.Subscriber('elisa3_robot_' + str(self.optitrack_id)+ "/odom", Odometry,
                                                    self.listen_robot_pose_callback)
        # self.listener_robot_pose = rospy.Subscriber("/odom", Odometry,
        #                                             self.listen_robot_pose_callback)
        self.publisher_motor_vlt = rospy.Publisher('elisa3_robot_' + str(self.optitrack_id)+'/mobile_base/cmd_vel',
                                                   Float64MultiArray)
        # self.publisher_motor_vlt = rospy.Publisher('elisa3_robot'+'/mobile_base/cmd_vel',
        #                                            Float64MultiArray)
        self.publisher_greenLed = rospy.Publisher('elisa3_robot_' + str(self.optitrack_id)+'/green_led',
                                                   Float64MultiArray)

        self.motor_vlt_msg = Float64MultiArray()
        self.green_led_msg = Float64MultiArray()

        self.std_speed = 0.045   #[m/s]
        self.std_vlt = 10

        self.error_angle = 0
        self.rel_move = 0
        self.corr_abs_move = 0

        self.start_pos = np.array([0.,0.])
        self.current_pos = np.array([0.,0.])
        self.start_orien = 0.
        self.current_orien = 0.

    def listen_optitrack_pose_callback(self,pose):
        # self.pose = np.array([float(pose.pose.position.x),float(pose.pose.position.y)])
        # self.orien = float(pose.pose.orientation.z)
        self.dummy = 0

    def listen_robot_pose_callback(self,odomMsg):
        self.pose = np.array([float(odomMsg.pose.pose.position.x),float(odomMsg.pose.pose.position.y)])
        # if self.controll_id == 1:
        #     print(self.pose)
        self.orien = float(odomMsg.pose.pose.orientation.z)
        self.odomMsg_time = odomMsg.header.stamp.secs

    def publish_motor_vlt(self, vlt = np.array([0,0])): #leftVoltage, rightVoltage
        # self.motor_vlt_msg.data = np.append(np.array([self.controll_id]), vlt)
        self.motor_vlt_msg.data = vlt
        self.publisher_motor_vlt.publish(self.motor_vlt_msg)

    def publish_greenLed(self, intensity = np.array([0])):
        self.green_led_msg.data = intensity
        self.publisher_greenLed.publish(self.green_led_msg)

    def move(self, rel_pol_move=np.array([0,0])): #[rho, phi]
        # turn right direction
        self.publish_motor_vlt(np.array([-self.std_vlt, self.std_vlt]))
        time.sleep((rel_pol_move[1]*wheel_distance/2)/self.std_speed)
        self.publish_motor_vlt()

        time.sleep(0.5)

        # move straight
        self.publish_motor_vlt(np.array([self.std_vlt, self.std_vlt]))
        time.sleep(rel_pol_move[0]/self.std_speed)
        self.publish_motor_vlt()

        time.sleep(0.5)

        self.current_pos = self.pose
        self.current_orien = self.orien

    def callibrate(self):
        call_time = 2.5

        # # relative callibration
        # start_time = time.time()
        # start_pose = self.pose
        # self.publish_motor_vlt(np.array([self.std_vlt, self.std_vlt]))
        # time.sleep(call_time)
        # end_time = time.time()
        # self.publish_motor_vlt()
        # end_pose = self.pose
        #
        # self.publish_motor_vlt(np.array([-self.std_vlt, -self.std_vlt]))
        # time.sleep(call_time)
        # self.publish_motor_vlt()
        #
        # norm_meas_move = normalize(end_pose-start_pose)
        # self.error_angle = angle_vectors(np.array([1,0]), norm_meas_move)
        # self.rel_move_diff = 2*wheel_distance * self.error_angle

        # abs callibration
        # start_time = time.time()
        start_time = self.odomMsg_time
        start_pose = self.pose
        self.publish_motor_vlt(np.array([self.std_vlt, self.std_vlt]))
        time.sleep(call_time)
        # end_time = time.time()
        self.publish_motor_vlt()
        end_pose = self.pose
        end_time = self.odomMsg_time
        meas_move = end_pose-start_pose
        meas_speed = np.linalg.norm(meas_move)/call_time

        # self.corr_abs_move = 1 - meas_speed / self.std_speed
        self.std_speed = meas_speed

        print('callibrated robot: ' + str(self.optitrack_id))
        # # test abs callibration
        # if meas_speed < self.std_speed:
        #     for i in range(0,20):
        #         self.std_vlt += 0.5
        #         start_pose = self.pose
        #         self.publish_motor_vlt(np.array([self.std_vlt,
        #                                          self.std_vlt]))
        #         # self.publish_motor_vlt(np.array([self.std_vlt, self.std_vlt]))
        #         time.sleep(call_time)
        #         end_time = time.time()
        #         self.publish_motor_vlt()
        #         end_pose = self.pose
        #
        #         meas_move = end_pose-start_pose
        #         meas_speed = np.linalg.norm(meas_move)/call_time
        #
        #         if np.linalg.norm(meas_speed - self.std_speed) < 0.0025:
        #             break

    def pendle_experiment(self):
        print('[' + str(self.optitrack_id)+'] - start location: ' + str(self.current_pos))
        print('[' + str(self.optitrack_id)+'] - start orientation: ' + str(self.current_orien))
        self.move(np.array([0.1, np.pi]))
        print('[' + str(self.optitrack_id)+'] - halfway location: ' + str(self.current_pos))
        print('[' + str(self.optitrack_id)+'] - halfway orientation: ' + str(self.current_orien))
        self.move(np.array([0.1, np.pi]))
        print('[' + str(self.optitrack_id)+'] - end location: ' + str(self.current_pos))
        print('[' + str(self.optitrack_id)+'] - end orientation: ' + str(self.current_orien))


class Nodes:
    def __init__(self, active_robots = ['0000']):

        rospy.init_node('python_node')
        self.nodes = {address: Node(mapper_dict[address]['optitrack_id'], mapper_dict[address]['elisa_id']) for
                      address in active_robots}

    def pendle_experiment(self):
        for address in self.nodes:
            self.nodes[address].pendle_experiment()

    def callibrate(self):
        for address in self.nodes:
            self.nodes[address].callibrate()








