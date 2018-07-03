#!/usr/bin/env python


'''
Goal of this is to waterfall out all 
the edges. May need to be adjusted for 
what our needs are. 
'''


import waterfall, numpy
from slam_map import SlamMap
import matplotlib.pyplot as plt
import numpy as np
from config import Config


class WallMap():

    def __init__(self):

        data_filepath = "smarthome_data/tokyo_slam_map.txt"
        config_filepath = "config/tokyo.txt"
        config = Config(config_filepath)
        slam_map = SlamMap(data_filepath, config)
        self.wf = waterfall.Waterfall(slam_map.map)


    def getMap(self):

        return self.wf.toNumpy()


    def displayIt(self, npmap):
        
        plt.imshow(np.transpose(npmap), cmap='hot', interpolation='nearest')
        axis = plt.gca()
        axis.set_ylim(axis.get_ylim()[::-1])
        plt.show()


#Testing
if __name__ == '__main__':

    wm = WallMap()
    newMap = wm.getMap()
    wm.displayIt(newMap)