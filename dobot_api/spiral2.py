import matplotlib.pyplot as pylot
from math import cos, sin, pi
from numpy import linspace

import numpy as np
import matplotlib.pyplot as plt



# iterations = 7
# # 0.6 
# points_count = 10000
# # step_div = 0.1 # 5.6
# # step_div = 0.3 # 2

# # step_div = 0.5 
# # step_div = 1/step
# diameter = 60
# step = 0.1

# diameter_wspl = diameter/60
# step_div = 0.58 * diameter_wspl / step # 1
# # ZADANY STEP 1 mm - step_div = 0.45
# # DIAMETER 60 mm - 

# theta = np.radians(np.linspace(0, 360*100*step_div, int(points_count*step_div)))
# r = (diameter/10)/80000*theta**2/(step_div*step_div)
# x_2 = r*np.cos(theta)
# y_2 = r*np.sin(theta)
# x_2_rev = np.flip(x_2)
# y_2_rev = np.flip(y_2)
# x = []
# y = []
# for i in range(1, iterations+1):
#     plt.plot(x_2_rev[100*i-100:100*i+1], y_2_rev[100*i-100:100*i+1])
#     # x.append(x_2_rev[100*i-100:100*i+1])
#     # y.append(y_2_rev[100*i-100:100*i+1])
# # return x, y
# plt.show()

def spiral(step, diameter, iterations):
    points_count = 10000
    diameter_wspl = diameter/60
    step_div = 0.58 * diameter_wspl / step # 1
    # ZADANY STEP 1 mm - step_div = 0.45
    # DIAMETER 60 mm - 

    theta = np.radians(np.linspace(0, 360*100*step_div, int(points_count*step_div)))
    r = (diameter/10)/80000*theta**2/(step_div*step_div)
    x_2 = r*np.cos(theta)
    y_2 = r*np.sin(theta)
    x_2_rev = np.flip(x_2)
    y_2_rev = np.flip(y_2)
    x = []
    y = []
    for i in range(1, iterations+1):
        plt.plot(x_2_rev[100*i-100:100*i+1], y_2_rev[100*i-100:100*i+1])
        # x.append(x_2_rev[100*i-100:100*i+1])
        # y.append(y_2_rev[100*i-100:100*i+1])
    # return x, y
    plt.show()

spiral(0.1, 60, 100)