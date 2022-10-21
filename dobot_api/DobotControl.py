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
from PyQt5.QtWidgets import QDialog, QApplication, QPushButton, QVBoxLayout

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
import random
from matplotlib.backends.qt_compat import QtWidgets
from matplotlib.backends.backend_qtagg import (
    FigureCanvas, NavigationToolbar2QT as NavigationToolbar)
from matplotlib.figure import Figure


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

        # moveX = 0;
        # moveY = 0;
        # moveZ = 10;
        # moveFlag = -1
        time.sleep(20)
        pos = dType.GetPose(api)
        print(pos)
        x = pos[0]
        y = pos[1]
        z = pos[2]
        rHead = pos[3]
        return x, y, z, rHead

    def spiral(self, step, diameter, iterations):
        """
        Returns trajectory points in the form of spiral

        :return:
        list x, y
        """

        points_count = 20000 
        points_factor = points_count/10000
        dia_factor = diameter/60
        step_div = 0.58 * dia_factor / step 

        theta = np.radians(np.linspace(0, 360*100*step_div, int(points_count*step_div*points_factor)))
        r = (diameter/10)/80000*theta**2/(step_div*step_div)

        x_2 = r*np.cos(theta)
        y_2 = r*np.sin(theta)
        x_2_rev = np.flip(x_2)
        y_2_rev = np.flip(y_2)
        x = []
        y = []
        
        iter_point_count = int(100*points_factor**2)
        for i in range(1, iterations+1):
            x.append(x_2_rev[iter_point_count*i-iter_point_count:iter_point_count*i+1])
            y.append(y_2_rev[iter_point_count*i-iter_point_count:iter_point_count*i+1])
        return x, y

    def triangle(self, step, diameter, iterations):
        """
        Returns trajectory points in the form of traingle
        :param step:
        :return:
         """

        x1 = np.array([0, -diameter/2, diameter/2, 0])
        y1 = np.array([-math.sqrt((diameter/2)**2 - (diameter/4)**2), diameter/2, diameter/2, -math.sqrt((diameter/2)**2 - (diameter/4)**2)+2*step])
        # TODO: Switch to numpy
        x = [[] for i in range(iterations)]
        y = [[] for i in range(iterations)]
        offset = 0
        for i in range(0, iterations):
            for j in range(0, len(x1)):
                x[i].append(x1[j]) if x1[j] == 0 else x[i].append(x1[j] - offset * 2) if x1[j] > 0 else x[i].append(
                    x1[j] + offset * 2)
                y[i].append(y1[j] + offset * 2) if x1[j] == 0 else y[i].append(y1[j] - offset) if y1[j] > 0 else y[i].append(
                    y1[j] + offset)
            offset += step
        return x, y

    def square(self, step, diameter, iterations):
        """
        Returns trajectory points in the form of square
        :param step:
        :return:
        """
        x1 = np.array([-diameter/2, -diameter/2, diameter/2, diameter/2, -diameter/2+step])
        y1 = np.array([-diameter/2, diameter/2, diameter/2, -diameter/2+step, -diameter/2+step])
        # TODO: Switch to numpy
        x = [[] for i in range(iterations)]
        y = [[] for i in range(iterations)]
        offset = 0
        for i in range(0,iterations):
            for j in range(0,len(x1)):
                x[i].append(x1[j]) if x1[j]==0 else x[i].append(x1[j]-offset) if x1[j]>0 else x[i].append(x1[j]+offset)
                y[i].append(y1[j]+offset) if x1[j]==0 else y[i].append(y1[j]-offset) if y1[j]>0 else y[i].append(y1[j]+offset)
            offset += step
        return x, y

    def execute_trajectory(self, x_pos, y_pos, api, zstep, tool_length):
        """
        Executes given trajectory via points
        :param x_pos:
        :param y_pos:
        :return:
        """
        pos = dType.GetPose(api)
        print(pos)
        rHead = 0
        # dType.SetCPCmd(api, 1, -20 + OFFSET_X, 0, -57 - tool_length, rHead, isQueued=1)[0]
        # Async CPC Motion
        for i in range(len(x_pos)):
            for j in range(len(x_pos[i])):
                # lastIndex = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVLXYZMode, x_2_list_popr[i] - x + 180, y_2_list_popr[i], 135 - 212 - 0.005*(i-1), rHead, isQueued = 1)[0]
                # lastIndex = dType.SetCPCmd(api, 1, x_pos[i][j] - 200 + OFFSET_X + WS_OFFSET_X, y_pos[i][j], -57 - tool_length - zstep * (i - 1), rHead, isQueued=1)[0]
                lastIndex = dType.SetCPCmd(api, 1, - x_pos[i][j] + pos[0], y_pos[i][j]*3/4 + pos[1], pos[2] - zstep * (i - 1), rHead, isQueued=1)[0]
                if j==250:
                    print(x_pos[i][j] + pos[0], y_pos[i][j]*3/4 + pos[1])

        # Start to Execute Command Queue
        dType.SetQueuedCmdStartExec(api)
        
        # Wait for Executing Last Command
        while lastIndex > dType.GetQueuedCmdCurrentIndex(api)[0]:
            dType.dSleep(100)

        # Stop to Execute Command Queued
        dType.SetQueuedCmdStopExec(api)

    def show_plot(self, function, step, diameter, iterations):
        if function == "Spiral":
            x, y = self.spiral(step, diameter, iterations)
        elif function == "Triangle":
            x, y = self.triangle(step, diameter, iterations)
        elif function == "Square":
            x, y = self.square(step, diameter, iterations)
        else:
            pass
        for i in range(iterations):
            plt.plot(x[i], y[i])
        plt.show()

    def emergency_stop(self, api):
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

    def connect_click_event(self):
        port = self.ui.port_line.text()
        baudrate = int(self.ui.baudrate_line.text())
        self.state = self.dobot.establish_connection(port=port, baudrate=baudrate, api=self.api)
        return self.state

    def home_click_event(self):
        if (self.state == dType.DobotConnect.DobotConnect_NoError):
            vel = int(self.ui.velocity_line.text())
            acc = int(self.ui.acceleration_line.text())
            self.dobot.home_setup(self.api, vel, acc)
        else:
            print("[ERROR] Robot not connected")

    def plan_click_event(self):
        x_list, y_list = None, None
        diameter = int(self.ui.diameter_line.text())
        xy_step = float(self.ui.xystep_line.text())
        iterations = int(self.ui.iterations_line.text())
        if self.ui.shapeComboBox.currentText() == "Spiral":
            x_list, y_list = self.dobot.spiral(step=xy_step, diameter=diameter, iterations=iterations)
        elif self.ui.shapeComboBox.currentText() == "Triangle":
            x_list, y_list = self.dobot.triangle(step=xy_step, diameter=diameter, iterations=iterations)
        elif self.ui.shapeComboBox.currentText() == "Square":
            x_list, y_list = self.dobot.square(step=xy_step, diameter=diameter, iterations=iterations)
        return x_list, y_list

    def execute_click_event(self):
        x_list, y_list = self.plan_click_event()
        zstep = float(self.ui.zstep_line.text())
        tool_length = int(self.ui.tool_length.text())
        if (self.state == dType.DobotConnect.DobotConnect_NoError):
            self.dobot.execute_trajectory(x_list, y_list, self.api, zstep, tool_length)
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
        iterations = int(self.ui.iterations_line.text())

        if self.ui.shapeComboBox.currentText() == "Spiral":
            self.dobot.show_plot("Spiral", xystep, diameter, iterations)
        elif self.ui.shapeComboBox.currentText() == "Triangle":
            self.dobot.show_plot("Triangle", xystep, diameter, iterations)
        elif self.ui.shapeComboBox.currentText() == "Square":
            self.dobot.show_plot("Square", xystep, diameter, iterations)
        else:
            pass



if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = DobotMainWindow()
    w.show()
    sys.exit(app.exec_())

# if __name__ == '__main__':
#     dobot = DobotControl()
#     api = dType.load()
#     state = dobot.establish_connection(port="COM3", baudrate=115200, api=api)
#
#     if (state == dType.DobotConnect.DobotConnect_NoError):
#         x, y, z, rHead = dobot.home_setup(api, 200, 200)
#         x_2_list_offset, y_2_list_offset = dobot.spiral(step=LOW_STEP, diameter=60)
#         #x_2_list_offset, y_2_list_offset = dobot.triangle(step=LOW_STEP)
#         dobot.execute_trajectory(x_2_list_offset, y_2_list_offset, api, 0.01, 15)
#     dType.DisconnectDobot(api)
