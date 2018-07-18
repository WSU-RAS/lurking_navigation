from __future__ import division

#!/usr/bin/env python
""" python lurking_ai.py ~/ras/src/smarthome_data/tokyo_sensors.txt ~/ras/src/lurking_navigation/config/tokyo.txt ~/ras/src/smarthome_data/tokyo_slam_map.txt 

lab:=True
 """


"""
    LurkingAI listens to sensor data and uses it to create a DynamicHeatmap
    object, acquire a weighted average, and create a path map.
    This class also imports the reachability map.
    Those four pieces of information are used to suggest an "optimal" location
    for Ras to be at every TIME_TICK.

    Requirements:
    - roscore must be running
    - The publisher must be running in another terminal
"""

import sys
import rospy
import numpy as np

from util.transform import transform_back_to_slam

from dynamic_heatmap import DynamicHeatmap
from dynamic_overlay import DynamicOverlay
from reachability_map import ReachabilityMap
from base_placement_algorithm import BasePlacer
from slam_map import SlamMap
from path_map import PathMap
import matplotlib.pyplot as plt
from weighted_average import WeightedAverage
from ras_msgs.msg import SensorPub
from ras_msgs.srv import Goto_xy
from config import Config
from wall_map import WallMap
from matplotlib.colors import LinearSegmentedColormap

from path_map import get_point_distance

TIME_TICK = 3    # in seconds
UPDATE_RAS = 3   # in seconds


class LurkingAI():
    """
        Subscribes to sensor data and publishes a landing location for Ras. 
    """

    def __init__(self, dynamic_heatmap, slam_data_filepath, config, simulated=False):
        self.map = None
        self.average_point = (0, 0)
        self.landing_zone = (0,0)
        self.slam_weight = 1.0
        self.wa_weight = 0.1
        self.path_weight = 1500.0

        self.config = config 
        self.map_width = config.map_width
        self.map_height = config.map_height
        self.map_resolution = config.map_resolution

        self.dynamic_heatmap = dynamic_heatmap
        self.slam_data_filepath = slam_data_filepath 
        self.slam_map = SlamMap(slam_data_filepath, config)
        self.reachability_map = ReachabilityMap(self.slam_map)
        self.simulated = simulated
        self.path_map_array = PathMap(self.dynamic_heatmap).get_as_array()

        # The historical heatmap doesn't decay values over time
        self.historical_heatmap = DynamicHeatmap(sensor_list_filepath, config)
        self.historical_pathmap = PathMap(
            self.historical_heatmap).get_as_array()

        if simulated is False:
            self.listener()     

    def _build_map(self):
        # find the value of each point
        placement_map = np.zeros_like(self.dynamic_heatmap.get_heatmap())

        for i in range(placement_map.shape[0]):
            for j in range(placement_map.shape[1]):
                # find the value of a given location
                placement_map[i, j] = self.find_value(i, j)

        # rescale to between zero and 1
        # note that all values in the map are less than zero
        amin = np.amin(placement_map)
        placement_map = np.subtract(placement_map, amin)
        amax = np.amax(placement_map)
        placement_map = np.true_divide(placement_map, amax)

        # apply reachability mask
        placement_map = self.apply_reachability_mask(placement_map)

        self.map = placement_map

    def apply_reachability_mask(self, placement_map):
        reachability_map = self.reachability_map.map

        masked_map = np.zeros_like(reachability_map, dtype="float")

        for i in range(reachability_map.shape[0]):
            for j in range(reachability_map.shape[1]):
                # skip values outside the placement map
                if i >= placement_map.shape[0] or j >= placement_map.shape[1]:
                    continue

                if reachability_map[i, j] > 0:
                    masked_map[i, j] = placement_map[i, j]

        return masked_map

    def find_value(self, i, j):
        """
        find the placement value at the given array location
        """
        # get weighted average value
        from_point = (i, j)
        to_point = self.average_point
        wa_value = -(get_point_distance(from_point, to_point)
                     ** 2) * self.wa_weight

        # get path map value
        path_value = -self.historical_pathmap[i, j] * self.path_weight

        return wa_value + path_value

    def get_best_point(self):
        """ get the location of the best point

        returns:
            tuple (x, y) - indecies of the best scoring map location
        """
        max_value = 0.0  # minimum value of the map is 0.0
        best_point = (-1, -1)

        for i in range(self.map.shape[0]):
            for j in range(self.map.shape[1]):
                if self.map[i, j] > max_value:
                    max_value = self.map[i, j]
                    best_point = (i, j)

        return best_point

    """
        Creates a listener node that acquires sensor data continuously. 
    """

    def listener(self):
        rospy.init_node('sensor_listener', anonymous=True)
        rospy.Timer(rospy.Duration(TIME_TICK), self.timer_callback)
        rospy.Timer(rospy.Duration(UPDATE_RAS), self.get_landing_zone)
        rospy.Subscriber("sensor_tripped", SensorPub, self.update_heatmap)
        rospy.spin()

    """
        Inform the heatmap handler that sensor was triggered
    """

    def update_heatmap(self, data):
        if self.simulated:
            self.dynamic_heatmap.update_heatmap(data.sensor_name)
            self.historical_heatmap.update_heatmap(data.sensor_name)
            self.historical_pathmap = PathMap(
                self.historical_heatmap).get_as_array()
        else:
            self.dynamic_heatmap.update_heatmap(data.name)
            self.historical_heatmap.update_heatmap(data.name)

        #print np.amax(self.historical_pathmap) 

        # display the other maps in relation to our heatmap 
        self._display_maps()

        # self.dynamic_heatmap.display_heatmap()   

    """
        Displays the pathmap, heatmaps, weighted_average
    """
    def _display_maps(self):
        plt.gcf().clear()
        np_heatmap = np.array(self.dynamic_heatmap.get_heatmap())
        axis = plt.gca()


        #path_map = PathMap(self.dynamic_heatmap)
        #path_array = path_map.get_as_array()

        #for i in self.path_map_array:
        pathmap = PathMap(self.dynamic_heatmap).get_as_array()

        #print np.amax(pathmap)
        

        plt.imshow(np.transpose(pathmap), cmap='magma', interpolation='nearest')


        plt.imshow(np.transpose(np_heatmap),
                   cmap=get_custom_colormap_green(), interpolation='nearest')

        # WEIGHTED AVERAGE 
        # MAKE SURE IS LANDING ZONE 
        weighted_average_point = axis.plot(self.landing_zone[0], self.landing_zone[1], 'bo')

        # Checks if our y-axis is aligned with the origin
        if (axis.get_ylim()[0] > axis.get_ylim()[1]):
            axis.set_ylim(axis.get_ylim()[::-1])

        fig = plt.gcf()
        fig.show()
        fig.canvas.draw()

        # also display map of best point and others like in base placement 

    """
        Every TIME_TICK, decay the heatmap by the HEATMAP_DECAY_STRENGTH. 
    """

    def timer_callback(self, data=None):
        self.dynamic_heatmap.decay_heatmap()

    """
        Using the heatmap, pathmap, and weighted average, returns an "optimal"
        landing zone for Ras at this current moment. 
    """

    def get_landing_zone(self, data=None):
        # Use the dynamic_heatmap to grab the weighted average
        weighted_average = WeightedAverage(
            self.dynamic_heatmap).get_weighted_average_point()
        self.average_point = weighted_average
        self.landing_zone = weighted_average

        self._build_map()

        self.landing_zone = self.get_best_point()
        self.dynamic_heatmap.mark_spot_on_map(self.landing_zone)

        # transform landing zone back to original slam map
        self.landing_zone = transform_back_to_slam(self.landing_zone, self.slam_map)

        self._move_ras_to(self.landing_zone)
        return self.landing_zone

    def _move_ras_to(self, landing_zone):
        rospy.wait_for_service('goto_xy')
        goto_spot = rospy.ServiceProxy('goto_xy', Goto_xy)

        # destination = self.convert_grid_to_meters(landing_zone)
        destination = landing_zone
        print "ras is going to", destination

        move = goto_spot(destination[0], destination[1])
        return move.response

    def convert_grid_to_meters(self, gridpoint):
        result = [0,1]
        result[0] = gridpoint[0] * self.map_resolution   
        result[1] = gridpoint[1] * self.map_resolution
        return result

def get_custom_colormap_green():
    cdict = \
            {
                'red': ((0.0, 0.0, 0.0),
                        (1.0, 0.0, 0.0)),
                'green': ((0.0, 0.0, 0.0),
                        (1.0, 1.0, 1.0)),
                'blue': ((0.0, 0.0, 0.0),
                        (1.0, 0.5, 0.5)),
                'alpha': ((0.0, 0.0, 0.0),
                        (0.1, 1.0, 1.0),
                        (1.0, 1.0, 1.0))
            }
    return LinearSegmentedColormap('alpha_red', cdict)

def get_custom_colormap_blue():
    cdict = \
            {
                'red': ((0.0, 0.0, 0.0),
                        (1.0, 0.0, 0.0)),
                'green': ((0.0, 0.0, 0.0),
                        (1.0, 0.0, 0.0)),
                'blue': ((0.0, 0.0, 0.0),
                        (1.0, 1.0, 1.0)),
                'alpha': ((0.0, 0.0, 0.0),
                        (0.1, 1.0, 1.0),
                        (1.0, 1.0, 1.0))
            }
    return LinearSegmentedColormap('alpha_red', cdict)


if __name__ == "__main__":
    # Acquire filepaths
    sensor_list_filepath = sys.argv[1]
    config_filepath = sys.argv[2]
    slam_map_filepath = sys.argv[3]
    config = Config(config_filepath)

    # Create a dynamic heatmap object
    dynamic_heatmap = DynamicHeatmap(sensor_list_filepath, config)

    # Create a LurkingAI object that sends information to the heatmap
    lurking_ai = LurkingAI(dynamic_heatmap, slam_map_filepath, config)
