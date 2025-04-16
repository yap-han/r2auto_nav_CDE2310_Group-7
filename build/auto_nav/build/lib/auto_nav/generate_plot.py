import numpy as np
import matplotlib.pyplot as plt
laser_range = np.loadtxt('lidar.txt')

"""
plt.figure()
plt.polar(np.arange(0,360,360/len(laser_range))/180*np.pi,laser_range)
plt.show()
"""

laser_range[laser_range==0] = np.nan
lr2i = np.nanargmax(laser_range)
lr2i
laser_range[lr2i]