import os
import sys
import threading
import DobotDllType as dType
import matplotlib
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import array as arr
# import pydobot
from serial.tools import list_ports
import math
import time
from PyQt5 import QtCore, QtGui, QtWidgets
from main_window import Ui_MainWindow


CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"}
OFFSET_X = 177
OFFSET_Y = 1
BIG_STEP = 10
MID_STEP = 20
LOW_STEP = 30
WS_OFFSET_X = 185
WS_OFFSET_Y = 4

class DobotControl:
    # def __init__(self):
    #     # Load Dll and get the CDLL object
    #     self.api = dType.load()

    def establish_connection(self, port, baudrate, api):
        """
        Establishes connection with Dobot Magician

        :param port:
        :param baudrate:
        :return:
        api
        state
        """
        # # Load Dll and get the CDLL object
        #api = dType.load()

        # Connect Dobot
        state = dType.ConnectDobot(api, port, baudrate)[0]
        print("Connect status:", CON_STR[state])
        return state

    def home_setup(self, api, vel, acc):
        """
        Returns current pose of end effector after homing
        :param api:
        :return:
        """
        # Clean Command Queued
        dType.SetQueuedCmdClear(api)

        # Async Motion Params Setting
        dType.SetHOMEParams(api, 200, 200, 200, 200, isQueued=1)
        dType.SetPTPCoordinateParams(api, vel, acc, vel, acc)
        dType.SetPTPJumpParams(api, 10, 200)
        dType.SetPTPCommonParams(api, 100, 100)
        dType.SetCPParams(api, acc/4, vel/4, acc/4, 0, 0)

        # Async Home
        dType.SetHOMECmd(api, temp=0, isQueued=1)

        moveX = 0;
        moveY = 0;
        moveZ = 10;
        moveFlag = -1
        time.sleep(20)
        pos = dType.GetPose(api)
        print(pos)
        x = pos[0]
        y = pos[1]
        z = pos[2]
        rHead = pos[3]
        return x, y, z, rHead


    def circle(self, diameter):
        theta = np.linspace( 0 , 2 * np.pi , 150 )
 
        radius = diameter/2
        x_list = []
        y_list = []
        for i in range(20):
            x_list.append((radius-0.5*i) * np.cos(theta))
            y_list.append((radius-0.5*i) * np.sin(theta))
        x = radius * np.cos( theta )
        y = radius * np.sin( theta )
        
        x2 = (radius-5) * np.cos( theta )
        y2 = (radius-5) * np.sin( theta )
        figure, axes = plt.subplots( 1 )
        
        # x_list_poprawione = []
        # for i in x_list:
        #     for j in x_list[i]:
        #         x_list_poprawione.append(x_list[i][j])
        # y_list_poprawione = []
        # for i in x_list:
        #     for j in y_list[i]:
        #         y_list_poprawione.append(y_list[i][j])
        # print(x_list)
        # print(y_list[0])
        x_list_poprawione = []
        for i in range(len(x_list)):
            for j in range(len(x_list[i])):
                x_list_poprawione.append(x_list[i][j])
        y_list_poprawione = []
        for i in range(len(y_list)):
            for j in range(len(y_list[i])):
                y_list_poprawione.append(y_list[i][j])
        # print(x_list_poprawione)
        # for i in range(len(x_list)):
        #     axes.plot(x_list[i], y_list[i])
        axes.plot(x_list_poprawione, y_list_poprawione)
        # axes.plot( x, y )
        # axes.plot( x2, y2 )
        axes.set_aspect( 1 )
        
        plt.show()
    
    def circle2(self, diameter):
        theta = np.linspace( 0 , 2 * np.pi , 150 )
 
        radius = diameter/2
        x = []
        y = []
        for i in range(10):
            x.append(radius-0.5*i * np.cos( theta ))
            y.append(radius-0.5*i * np.cos( theta ))
        
        figure, axes = plt.subplots( 1 )
        
        print(x)
        print(y)
        axes.plot( x, y )
        axes.set_aspect( 1 )
        
        plt.show()

    def execute_trajectory(self, x_pos, y_pos, api, zstep, tool_length):
        """
        Executes given trajectory via points
        :param x_pos:
        :param y_pos:
        :return:
        """
        rHead = 0
        dType.SetCPCmd(api, 1, -20 + OFFSET_X, 0, -57 - tool_length, rHead, isQueued=1)[0]
        # Async PTP Motion
        for i in range(0, len(x_pos), 1):
            """if i % 2 == 0:
                offset = 50
            else:
                offset = -50"""
            # lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_2_list_popr[i] - x + 180, y_2_list_popr[i], 135 - 212 - 0.005*(i-1), rHead, isQueued = 1)[0]
            lastIndex = dType.SetCPCmd(api, 1, x_pos[i] - 200 + OFFSET_X, y_pos[i]*3/4, -57 - tool_length - zstep * (i - 1), rHead, isQueued=1)[0]

        # Start to Execute Command Queue
        dType.SetQueuedCmdStartExec(api)
        
        # Wait for Executing Last Command
        while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(100)

        # Stop to Execute Command Queued
        dType.SetQueuedCmdStopExec(api)


if __name__ == '__main__':
    dobot = DobotControl()
    api = dType.load()
    state = dobot.establish_connection(port="COM3", baudrate=115200, api=api)

    if (state == dType.DobotConnect.DobotConnect_NoError):
        x, y, z, rHead = dobot.home_setup(api, 200, 200)
        x_2_list_offset, y_2_list_offset = dobot.circle(diameter=60)
        #x_2_list_offset, y_2_list_offset = dobot.triangle(step=LOW_STEP)
        dobot.execute_trajectory(x_2_list_offset, y_2_list_offset, api, 0.01, 15)
    dType.DisconnectDobot(api)

