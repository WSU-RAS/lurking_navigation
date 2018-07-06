#!/usr/bin/env python


import sys
from Queue import Queue

import numpy as np
import matplotlib.pyplot as plt


class CrampedMap:
    """ Cramped map

    given a reachability finds all the places that are cramped.

    Adapted from reachability map. 

    """

    def __init__(self, reachability_map):
        # constants
        # self.start_x = 45
        # self.start_y = 30
        self.kernel_size = 4  # size from edge to center of kernel
        # self.impassability_cutoff = 70, not needed for cramped spaces. 

        # variables
        self.map = None

        # build the map
        self._build_map(reachability_map)

    def _build_map(self, reachability_map):
        """ build map

        inputs:
            reachabilty map - completed reachability map

        """
        # set up
        kernel_size = self.kernel_size

        r_map = reachability_map
        # cramped_map = np.zeros_like(r_map, dtype=int)
        # cramped_map = np.zeros_like()
        # cramped_map = np.full_like(r_map, 1, dtype=int) #Fill it all as not cramped. 

        cramped_map = np.copy(r_map)

        queue = Queue()

        #Create a que to look at all the reachable spots from the r_map. 1 = reachable, 0 = not reachable
        for i in range(r_map.shape[0]):
            for j in range(r_map.shape[1]):
                if r_map[i, j] == 1: 
                    queue.put((i, j))

        while queue.empty() is False:
            # get next element to check
            current_point = queue.get()
            invalid = False

            # check if this element is valid
            for i in range(current_point[0] - kernel_size, current_point[0] + kernel_size + 1):
                for j in range(current_point[1] - kernel_size, current_point[1] + kernel_size + 1):
                    # check if this index is valid
                    if i > 0 and i < r_map.shape[0] and j > 0 and j < r_map.shape[1]:
                        # check if this index violates the cutoff for being impassable
                        if r_map[i, j] != 1:
                            invalid = True

                    else:
                        invalid = True


            if invalid is True:
                continue

            # element is valid
            # update new values
            for i in range(current_point[0] - kernel_size, current_point[0] + kernel_size + 1):
                for j in range(current_point[1] - kernel_size, current_point[1] + kernel_size + 1):
                    # check if this index is valid
                    if i > 0 and i < cramped_map.shape[0] and j > 0 and j < cramped_map.shape[1]:
                        # update value
                        cramped_map[i, j] = 0

        self.map = cramped_map

    def display_as_heatmap(self):
        plt.imshow(np.transpose(self.map), cmap='hot', interpolation='nearest')

        # flip y axis
        axis = plt.gca()
        axis.set_ylim(axis.get_ylim()[::-1])

        plt.show()
