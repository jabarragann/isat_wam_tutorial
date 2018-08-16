#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import time
import socket
import numpy as np
import os


def pprint(name, l):
    l_str = ''.join(['%+.6f ' % item for item in l])
    print name + ': ' + l_str

class WAM:
    def __init__(self):
        # init node for wam_node 
        rospy.init_node('wam_node')   
        self.rate = rospy.Rate(2)

        
    def move_wam(self,key_pressed):
        #Check if the key is in the key mapping, if not don't move
        if len(key_pressed) == 0 or not key_mapping.has_key(key_pressed):
            return 

        #Get current joint positions
        pac = self.query_joint_pose()
        current_joints_dict = self.decode_joint_pac(pac)

        old_joints_position = [ current_joints_dict['j1'],current_joints_dict['j2'], current_joints_dict['j3'],\
                             current_joints_dict['j4'],current_joints_dict['j5'], current_joints_dict['j6'], current_joints_dict['j7']]

        #Update position
        joint_to_move = key_mapping[ key_pressed ][0] 
        sign = key_mapping[ key_pressed ][1]

        if   (sign == '+'):
            current_joints_dict[joint_to_move] += 0.07
        elif (sign == '-'):
            current_joints_dict[joint_to_move] -= 0.07
        else:
            return

        #Move Joint
        new_joints_position = [ current_joints_dict['j1'],current_joints_dict['j2'], current_joints_dict['j3'],\
                             current_joints_dict['j4'],current_joints_dict['j5'], current_joints_dict['j6'], current_joints_dict['j7']]

        print("Key pressed: {:s}".format(key_pressed) )
        pprint("old joint pose: ", old_joints_position)
        pprint("new joint pose: ", new_joints_position)

        self.goto_joint(new_joints_position)

    def receive_keyboard_command(self,msg):
        self.keyboard_command = msg.data[0]


    def clean_shutdown(self):
        """
        Exits example cleanly by moving head to neutral position and
        maintaining start state
        """
        print("\nExiting moveWAM_BRIDGE()...")
        if self.socket:
            self.go_home()
            self.socket.close()
            print ("Socket closed properly...")

    def init_socket(self, host, port, buflen):
        # init socket with Multimodal.exe in Windows C++
        if host == 'local_file':
            file = open("/home/santi/ros_workspaces/kinetic_ws/src/wam_simple_client/src/WAM_IP.txt", "r") 
            host = file.read()
            print ("recovered WAM IP %s from local file..." % host)

        # create a socket object
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.buflen = buflen

        # connection to hostname on the port.
        re_try = 1
        while re_try and not rospy.is_shutdown():
            try:
                self.socket.connect((host, port))
                re_try = 0
            except:
                print("Connection failed. Retry after 0.1 seconds")
                re_try = 1
                time.sleep(0.1)
                continue

        print ("Built socket connection with WAM PC...")
        print ("Heartbeat started to get real-time robot joint positions...")
        print ("Wait for path planning result to move the robot...")
    
    def query_joint_pose(self):
        self.socket.send('2')
        time.sleep(0.01)
        pac = self.socket.recv(self.buflen)
        return pac

    def query_cart_pose(self):
        self.socket.send('9')
        time.sleep(0.01)
        pac = self.socket.recv(self.buflen)
        return pac

    def goto_joint(self, targetJp):
        # publish the command to go to joint
        assert (len(targetJp) == 7)
        msg = '7 ' + ''.join([str(i)+' ' for i in targetJp])
        self.socket.send(msg)
        time.sleep(0.01)
        pac = self.socket.recv(self.buflen)
        assert (pac == 'complete')
        return pac 

    def goto_XYZ(self, targetCp, fixQuat):
        assert (len(targetCp) == 3)
        
        msg = '8 ' + ''.join([str(i)+' ' for i in targetCp])
        msg += '%i ' % fixQuat
        self.socket.send(msg)
        time.sleep(0.01)
        pac = self.socket.recv(self.buflen)
        assert (pac == 'complete')
        return pac 

    def go_home(self):
        print("go home WAM you are drunk!!!")
        self.socket.send('4')
        time.sleep(1)
            
    def decode_joint_pac(self, pac):
        pac_split = pac.split(' ')
        joints = []
        for i, item in enumerate(pac_split):
            if i == 7 or i>7:
                continue

            joint_float = float(item)
            joints.append(joint_float)

        joints_dict = {}
        joints_dict['j1'] = joints[0]
        joints_dict['j2'] = joints[1]
        joints_dict['j3'] = joints[2]
        joints_dict['j4'] = joints[3]
        joints_dict['j5'] = joints[4]
        joints_dict['j6'] = joints[5]
        joints_dict['j7'] = joints[6]

        return joints_dict

    def decode_pose_pac(self, pac):
        pac_split = pac.split(' ')
        pose = []
        for i, item in enumerate(pac_split):
            if i == 0 or i>7:
                continue

            pos_float = float(item)
            pose.append(pos_float)

        pos_dict = {}
        pos_dict['x'] = pose[0]
        pos_dict['y'] = pose[1]
        pos_dict['z'] = pose[2]
        pos_dict['q_w'] = pose[3]
        pos_dict['q_x'] = pose[4]
        pos_dict['q_y'] = pose[5]
        pos_dict['q_z'] = pose[6]
        return pos_dict

    def run(self):
        rospy.on_shutdown(self.clean_shutdown)
        while not rospy.is_shutdown():

            pac = self.query_joint_pose()
            joints_dict = self.decode_joint_pac(pac)
            print("current joint pose")
            print("j1:{:+3.3f} j2:{:+3.3f} j3:{:+3.3f} j4:{:+3.3f} j5:{:+3.3f} j6:{:+3.3f}".format(\
             joints_dict['j1'], joints_dict['j2'],joints_dict['j3'],joints_dict['j4'],joints_dict['j5'],joints_dict['j6']))

            pac  = self.query_cart_pose()
            pose_dict = self.decode_pose_pac(pac)
            print("current cart pose")
            print("xp:{:+3.3f} yp:{:+3.3f} zp:{:+3.3f} wq:{:+3.3f} xq:{:+3.3f} yq:{:+3.3f} zq:{:+3.3f}\n\n".format(\
             pose_dict['x'], pose_dict['y'],pose_dict['z'],pose_dict['q_w'],pose_dict['q_x'],pose_dict['q_y'],pose_dict['q_z']))


            self.rate.sleep()
            
            if not rospy.is_shutdown():
                os.system('clear')
            
        rospy.signal_shutdown("run() finished...")

#Global Variables
key_mapping = { '4':['j1','+'], 'r':['j1','-'],
                '5':['j2','+'], 't':['j2','-'],
                '6':['j3','+'], 'y':['j3','-'],
                '7':['j4','+'], 'u':['j4','-'],
                '8':['j5','+'], 'i':['j5','-'],
                '9':['j6','+'], 'o':['j6','-'],
                '0':['j7','+'], 'p':['j7','-']}

joint_limit_values = { 'j1':[-0.677, 0.6909],
                	   'j2':[-1.553, -0.736],
                	   'j3':[-0.9142, 1.0891],
                	   'j4':[1.46, 2.997],
                	   'j5':[-0.632, 0.8625],
                	   'j6':[-0.4389, 0.6088],
                	   'j7':[-1.5606, 1.096]}
        
if __name__ == '__main__':
    try:
        wam = WAM()
        wam.init_socket(host='local_file', port=4000, buflen=256)
        wam.run()
    except KeyboardInterrupt:
        print("Ok ok, keyboard interrupt, quitting")
        sys.exit(1)
    else:
        print("Normal termination")
        sys.exit(2)
