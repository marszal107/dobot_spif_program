import matplotlib
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
import array as arr
import pydobot
from serial.tools import list_ports
import math
#import cv2 


def spirala():
    theta = np.radians(np.linspace(50, 360*5, 10000))
    r = theta**2
    x_2 = r*np.cos(10*theta)/30
    y_2 = r*np.sin(10*theta)/30

    plt.figure(figsize=[10, 10])
    plt.plot(x_2, y_2)

    x_2_list = x_2.tolist()
    y_2_list = y_2.tolist()

    xy_2_list = []

    for i in range(len(x_2_list)):
        xy_2_list.append([x_2_list[i], 
                        y_2_list[i]])
        i+=i

    x_2_list_popr = []
    y_2_list_popr = []
    for i in range(0,len(x_2_list)-1,1):
        x_2_list_popr.append((x_2_list[i] + 250)) 
        y_2_list_popr.append((y_2_list[i] + 250)) 
        i+=i
    x_2_list_popr.reverse()
    y_2_list_popr.reverse()
    print(x_2_list_popr[1] + 270 -200)
    return xy_2_list

def triangle():
    """
    Returns trajectory points in the form of traingle
    :param step:
    :return:
     """
    x1=[0, -30, 30]
    y1=[-26, 30, 30]
    x2=[]
    y2=[]
    offset = 0
    for i in range(0,20):
        for j in range(0,len(x1)):
            x2.append(x1[j]) if x1[j]==0 else x2.append(x1[j]-offset*2) if x1[j]>0 else x2.append(x1[j]+offset*2)
            y2.append(y1[j]+offset*2) if x1[j]==0 else y2.append(y1[j]-offset) if y1[j]>0 else y2.append(y1[j]+offset)
        offset+=0.5
    print(x2)
    print(y2)
    plt.figure(figsize=[10, 10])
    plt.plot(x2, y2)

def connect_to_robot():
    available_ports = list_ports.comports()
    print(f'available ports: {[x.device for x in available_ports]}')
    port = available_ports[0].device

    device = pydobot.Dobot(port=port, verbose=True)

    robot_responses = []
    xy_2_list = spirala()

    for i in range(len(xy_2_list)):
        z = 0
        r = 0
        xy_cords = xy_2_list[i]
        device.move_to(xy_cords[0], xy_cords[1], z, r, wait=True)
        (x, y, z, r, j1, j2, j3, j4) = device.pose()
        print(f'x:{x} y:{y} z:{z} j1:{j1} j2:{j2} j3:{j3} j4:{j4}')
        robot_responses.append(x, y)
        i+=i

    device.close()

if __name__ == "__main__":
    #spirala()
    triangle()
    plt.show()
#connect_to_robot()
"""
errors_list = []

for i in range(len(robot_responses)):
    xy_cords = xy_2_list[i]
    robot_response = robot_responses[i]
    errors_list.append([((xy_cords[0] - robot_response[0]) / xy_cords[0]), 
                        ((xy_cords[1] - robot_response[1]) / xy_cords[1])])
    i+=i

x_robot_responses = []
y_robot_responses = []

for i in range(len(robot_responses)):
    robot_response = robot_responses[i]
    x_robot_responses.append(robot_response[0])
    y_robot_responses.append(robot_response[1])
    i+=i

plt.figure(figsize=[10, 10])
plt.plot(x_robot_responses, y_robot_responses)
"""


