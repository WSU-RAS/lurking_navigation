

import sys
from Queue import Queue

import numpy as np
import matplotlib.pyplot as plt

from slam_map import SlamMap


class ReachabilityMap:
    """ Reachability map

    given a slam map finds all the places that are reachable from a set position inside the map

    """

    def __init__(self, slam_map):
        # constants
        self.start_x = 45
        self.start_y = 30
        self.kernel_size = 2  # size from edge to center of kernel
        self.impassability_cutoff = 70

        # variables
        self.map = None

        # build the map
        self._build_map(slam_map)

    def _build_map(self, slam_map):
        """ build map

        inputs:
            slam_map - completed slam map

        """
        # set up
        kernel_size = self.kernel_size

        s_map = slam_map.map
        reachability_map = np.zeros_like(s_map, dtype=int)

        queue = Queue()
        queue.put((self.start_x, self.start_y))

        while queue.empty() is False:
            # get next element to check
            current_point = queue.get()
            invalid = False

            # check if this element is valid
            for i in range(current_point[0] - kernel_size, current_point[0] + kernel_size + 1):
                for j in range(current_point[1] - kernel_size, current_point[1] + kernel_size + 1):
                    # check if this index is valid
                    if i > 0 and i < s_map.shape[0] and j > 0 and j < s_map.shape[1]:
                        # check if this index violates the cutoff for being impassable
                        if s_map[i, j] > self.impassability_cutoff:
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
                    if i > 0 and i < reachability_map.shape[0] and j > 0 and j < reachability_map.shape[1]:
                        # add to queue if not already
                        if reachability_map[i, j] == 0:
                            queue.put((i, j))

                        # update value
                        reachability_map[i, j] = 1

        self.map = reachability_map

    def display_as_heatmap(self):
        plt.imshow(np.transpose(self.map), cmap='hot', interpolation='nearest')

        # flip y axis
        axis = plt.gca()
        axis.set_ylim(axis.get_ylim()[::-1])

        plt.show()


def main():
    slam_map_filepath = sys.argv[1]

    slam_map = SlamMap(slam_map_filepath)
    reachability_map = ReachabilityMap(slam_map)

    reachability_map.display_as_heatmap()


if __name__ == "__main__":
    main()
