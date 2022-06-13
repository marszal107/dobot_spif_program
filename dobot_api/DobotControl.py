import os
import sys
import threading
import DobotDllType as dType
import matplotlib
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import array as arr
import pydobot
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
WS_OFFSET_X = 245
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

    def spiral(self, step, diameter):
        """
        Returns trajectory points in the form of spiral

        :return:
        list x, y
        """
        theta = np.radians(np.linspace(50, 360*5, 10000))
        r = theta**2
        x_2 = r*np.cos(1/step*theta)/(diameter/2)
        y_2 = r*np.sin(1/step*theta)/(diameter/2)

        # plt.figure(figsize=[5, 5])
        # plt.plot(x_2, y_2)

        x_2_list = x_2.tolist()
        y_2_list = y_2.tolist()

        # xy_2_list = []
        #
        # for i in range(len(x_2_list)):
        #     xy_2_list.append([x_2_list[i],
        #                     y_2_list[i]])
        #     i+=i

        x_2_list_offset = []
        y_2_list_offset = []
        for i in range(0,len(x_2_list)-1,1):
            x_2_list_offset.append(x_2_list[i] + WS_OFFSET_X)
            y_2_list_offset.append(y_2_list[i] + WS_OFFSET_Y)
            i+=i
        x_2_list_offset.reverse()
        y_2_list_offset.reverse()
        return x_2_list_offset, y_2_list_offset

    def spiral_plot(self, step, diameter):
        """
        Returns trajectory points in the form of spiral

        :return:
        list x, y
        """
        theta = np.radians(np.linspace(50, 360 * 5, 10000))
        r = theta ** 2
        x_2 = r * np.cos(1 / step * theta) / (diameter / 2)
        y_2 = r * np.sin(1 / step * theta) / (diameter / 2)

        plt.figure(figsize=[5, 5])
        plt.plot(x_2, y_2)

        x_2_list = x_2.tolist()
        y_2_list = y_2.tolist()

        return x_2_list, y_2_list

    def triangle(self, step, diameter):
        """
        Returns trajectory points in the form of traingle
        :param step:
        :return:
         """

        x1 = [0, -diameter/2, diameter/2]
        y1 = [-math.sqrt((diameter/2)**2 - (diameter/4)**2), diameter/2, diameter/2]
        print(y1)
        x2 = []
        y2 = []
        offset = 0
        for i in range(0, 20):
            for j in range(0, len(x1)):
                x2.append(x1[j]) if x1[j] == 0 else x2.append(x1[j] - offset * 2) if x1[j] > 0 else x2.append(
                    x1[j] + offset * 2)
                y2.append(y1[j] + offset * 2) if x1[j] == 0 else y2.append(y1[j] - offset) if y1[j] > 0 else y2.append(
                    y1[j] + offset)
            offset += step
        #print(x2)
        #print(y2)
        # plt.figure(figsize=[5, 5])
        # plt.plot(x2, y2)
        x_2_list_offset = []
        y_2_list_offset = []
        for i in range(0, len(x2) - 1, 1):
            x_2_list_offset.append((x2[i] + WS_OFFSET_X))
            y_2_list_offset.append(y2[i])
            i += i
        return x_2_list_offset, y_2_list_offset

    def triangle_plot(self, step, diameter):
        """
        Returns trajectory points in the form of traingle
        :param step:
        :return:
         """

        x1 = [0, -diameter/2, diameter/2]
        y1 = [-math.sqrt((diameter/2)**2 - (diameter/4)**2), diameter/2, diameter/2]
        print(y1)
        x2 = []
        y2 = []
        offset = 0
        for i in range(0, 20):
            for j in range(0, len(x1)):
                x2.append(x1[j]) if x1[j] == 0 else x2.append(x1[j] - offset * 2) if x1[j] > 0 else x2.append(
                    x1[j] + offset * 2)
                y2.append(y1[j] + offset * 2) if x1[j] == 0 else y2.append(y1[j] - offset) if y1[j] > 0 else y2.append(
                    y1[j] + offset)
            offset += step
        #print(x2)
        #print(y2)
        plt.figure(figsize=[5, 5])
        plt.plot(x2, y2)
        return x2, y2

    def square(self, step, diameter):
        """
        Returns trajectory points in the form of square
        :param step:
        :return:
        """
        x1=[-diameter/2, -diameter/2, diameter/2, diameter/2]
        y1=[-diameter/2, diameter/2, diameter/2, -diameter/2]
        x2=[]
        y2=[]
        offset = 0
        for i in range(0,20):
            for j in range(0,len(x1)):
                x2.append(x1[j]) if x1[j]==0 else x2.append(x1[j]-offset) if x1[j]>0 else x2.append(x1[j]+offset)
                y2.append(y1[j]+offset) if x1[j]==0 else y2.append(y1[j]-offset) if y1[j]>0 else y2.append(y1[j]+offset)
            offset += step
        #print(x2)
        #print(y2)
        # plt.figure(figsize=[5, 5])
        # plt.plot(x2, y2)
        x_2_list_offset = []
        y_2_list_offset = []
        for i in range(0, len(x2) - 1, 1):
            x_2_list_offset.append(x2[i] + WS_OFFSET_X)
            y_2_list_offset.append(y2[i])
            i += i
        return x_2_list_offset, y_2_list_offset

    def square_plot(self, step, diameter):
        """
        Returns trajectory points in the form of square
        :param step:
        :return:
        """
        x1=[-diameter/2, -diameter/2, diameter/2, diameter/2]
        y1=[-diameter/2, diameter/2, diameter/2, -diameter/2]
        x2=[]
        y2=[]
        offset = 0
        for i in range(0,20):
            for j in range(0,len(x1)):
                x2.append(x1[j]) if x1[j]==0 else x2.append(x1[j]-offset) if x1[j]>0 else x2.append(x1[j]+offset)
                y2.append(y1[j]+offset) if x1[j]==0 else y2.append(y1[j]-offset) if y1[j]>0 else y2.append(y1[j]+offset)
            offset += step
        #print(x2)
        #print(y2)
        plt.figure(figsize=[5, 5])
        plt.plot(x2, y2)
        return x2, y2

    def execute_trajectory(self, x_pos, y_pos, api, zstep):
        """
        Executes given trajectory via points
        :param x_pos:
        :param y_pos:
        :return:
        """
        rHead = 0
        # Async PTP Motion
        for i in range(0, len(x_pos), 1):
            """if i % 2 == 0:
                offset = 50
            else:
                offset = -50"""
            # lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_2_list_popr[i] - x + 180, y_2_list_popr[i], 135 - 212 - 0.005*(i-1), rHead, isQueued = 1)[0]
            lastIndex = dType.SetCPCmd(api, 1, x_pos[i] - 200 + OFFSET_X, y_pos[i], 135 - 212 - zstep * (i - 1), rHead, isQueued=1)[0]

        # Start to Execute Command Queue
        dType.SetQueuedCmdStartExec(api)

        # Wait for Executing Last Command
        while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(100)

        # Stop to Execute Command Queued
        dType.SetQueuedCmdStopExec(api)

    def show_plot(self, function, step, diameter):
        if function == "Spiral":
            plt.plot(self.spiral_plot(step, diameter))
            plt.show()
        elif function == "Triangle":
            plt.plot(self.triangle_plot(step, diameter))
            plt.show()
        elif function == "Square":
            plt.plot(self.square_plot(step, diameter))
            plt.show()
        else:
            pass

    def emergency_stop(self, api):
        #SetQueuedCmdForceStopExec
        dType.SetQueuedCmdStopExec(api)
        return None


class DobotMainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        self.api = dType.load()
        self.dobot = DobotControl()
        self.state = None
        super(DobotMainWindow, self).__init__(parent=parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.retranslateUi(self)
        self.ui.connect_button.clicked.connect(self.connect_click_event)
        self.ui.home_button.clicked.connect(self.home_click_event)
        self.ui.plan_button.clicked.connect(self.plan_click_event)
        self.ui.execute_button.clicked.connect(self.execute_click_event)
        self.ui.estop_button.clicked.connect(self.estop_click_event)
        self.ui.show_button.clicked.connect(self.show_click_event)

    def home_click_event(self):
        if (self.state == dType.DobotConnect.DobotConnect_NoError):
            vel = int(self.ui.velocity_line.text())
            acc = int(self.ui.acceleration_line.text())
            self.dobot.home_setup(self.api, vel, acc)
        else:
            print("[ERROR] Robot not connected")

    def connect_click_event(self):
        port = self.ui.port_line.text()
        baudrate = int(self.ui.baudrate_line.text())
        self.state = self.dobot.establish_connection(port=port, baudrate=baudrate, api=self.api)
        return self.state

    def plan_click_event(self):
        x_list, y_list = None, None
        diameter = float(self.ui.diameter_line.text())
        xy_step = float(self.ui.xystep_line.text())
        if self.ui.shapeComboBox.currentText() == "Spiral":
            x_list, y_list = self.dobot.spiral(step=xy_step, diameter=diameter)
        elif self.ui.shapeComboBox.currentText() == "Triangle":
            x_list, y_list = self.dobot.triangle(step=xy_step, diameter=diameter)
        elif self.ui.shapeComboBox.currentText() == "Square":
            x_list, y_list = self.dobot.square(step=xy_step, diameter=diameter)
        return x_list, y_list

    def execute_click_event(self):
        x_list, y_list = self.plan_click_event()
        zstep = float(self.ui.zstep_line.text())
        if (self.state == dType.DobotConnect.DobotConnect_NoError):
            self.dobot.execute_trajectory(x_list, y_list, self.api, zstep)
        else:
            print("[ERROR] Robot not connected")
        #dType.DisconnectDobot(self.api)

    def estop_click_event(self):
        if (self.state == dType.DobotConnect.DobotConnect_NoError):
            self.dobot.emergency_stop(self.api)
        else:
            print("[ERROR] Robot not connected")

    def show_click_event(self):
        xystep = float(self.ui.xystep_line.text())
        diameter = float(self.ui.diameter_line.text())

        if self.ui.shapeComboBox.currentText() == "Spiral":
            self.dobot.show_plot("Spiral", xystep, diameter)
        elif self.ui.shapeComboBox.currentText() == "Triangle":
            self.dobot.show_plot("Triangle", xystep, diameter)
        elif self.ui.shapeComboBox.currentText() == "Square":
            self.dobot.show_plot("Square", xystep, diameter)
        else:
            pass


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = DobotMainWindow()
    w.show()
    sys.exit(app.exec_())

# if __name__ == '__main__':
#     dobot = DobotControl()
#     api, state = dobot.establish_connection(port="COM3", baudrate=115200)
#
#     if (state == dType.DobotConnect.DobotConnect_NoError):
#         x, y, z, rHead = dobot.home_setup(api)
#         x_2_list_offset, y_2_list_offset = dobot.spiral(step=LOW_STEP)
#         #x_2_list_offset, y_2_list_offset = dobot.triangle(step=LOW_STEP)
#         dobot.execute_trajectory(x_2_list_offset, y_2_list_offset)
#     dType.DisconnectDobot(api)
