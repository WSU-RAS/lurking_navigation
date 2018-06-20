
import sys

import numpy as np
import matplotlib.pyplot as plt

from config import Config
from heatmap import get_sensor_map, DataLine
from slam_map import SlamMap
from path_map import get_point_distance
from reachability_map import ReachabilityMap
from base_placement_algorithm import BasePlacer

from util.shortest_paths import ShortestPaths

class Simulator:
    """ Simulator environment for playing back sensor data

    We want to take in a file of sensor triggers and play it back to both 
    run our real time algorithms and evaluate algorithms against battery life
    and DTD (Distance to delivery)

    """

    def __init__(self, sensor_map_filepath, previous_sensor_data_filepath, slam_map_filepath, config_filepath):

        # setup variables
        self.config = Config(config_filepath)
        self.sensor_map, self.ignored_sensors = get_sensor_map(sensor_map_filepath)
        self.slam_map = SlamMap(slam_map_filepath, self.config)
        self.reachability_map = ReachabilityMap(self.slam_map)

        self.shortest_paths = ShortestPaths(self.reachability_map.map)

        # setup map to be tested
        # TODO make this more flexible
        self.base_placer = BasePlacer(slam_map_filepath, sensor_map_filepath, previous_sensor_data_filepath, self.config)

    def run_test(self, current_sensor_data_filepath):
        data_file = open(current_sensor_data_filepath, "r")

        chosen_point = self.base_placer.get_best_point()

        distances = []
        distance = self.step(chosen_point, data_file)

        while distance is not None:
            if distance < 9000:
                distances.append(distance)
            distance = self.step(chosen_point, data_file)

        np_distances = np.array(distances)
        average = np.average(np_distances)
        maximum_distance = np.amax(np_distances)

        print "average distance: ", average
        print "maximum distance: ", maximum_distance
        plt.show()

    def step(self, chosen_point, data_file):
        """ run a single step of the simulation

        in this case get the next sensor trigger and tell how far it is from the given map value

        """

        point, time = self._get_next_point(data_file, self.sensor_map, self.ignored_sensors)

        if point is None:
            return None

        # apply offset
        point_x = int(point[0] / self.config.map_resolution + self.config.slam_origin[0])
        point_y = int(point[1] / self.config.map_resolution + self.config.slam_origin[1])

        point = (point_x, point_y)

        # get distance
        distance, path = self.shortest_paths.run_shortest_paths(point, chosen_point)
        
        # check we actually got a distance.
        if distance is None:
            return 999999.9

        distance = distance * self.config.map_resolution

        print distance

        # display the points and slam map
        plt.imshow(np.transpose(self.slam_map.map), cmap='hot', interpolation='nearest')

        x, y = zip(*path)
        plt.scatter(y, x)

        plt.plot(point[0], point[1], 'bo')
        plt.plot(chosen_point[0], chosen_point[1], 'go')

        axis = plt.gca()
        if axis.get_ylim()[0] > axis.get_ylim()[1]:
            axis.set_ylim(axis.get_ylim()[::-1])

        plt.pause(0.0)
        plt.gcf().clear()

        return distance

    def _get_next_point(self, smarthome_data_file, sensor_map, ignored_sensors):
        while True:
            next_line = smarthome_data_file.readline()

            # check if the file is done
            if len(next_line) == 0:
                return None, None

            data = DataLine(next_line)

            if data.sensor_type == "Control4-Motion" and data.message == "ON" and data.sensor_name not in ignored_sensors:
                point = sensor_map[data.sensor_name]
                time = data.time
                return point, time

def main():
    slam_data_filepath = sys.argv[1]
    sensor_list_filepath = sys.argv[2]
    previous_smarthome_data_filepath = sys.argv[3]
    current_smarthome_data_filepath = sys.argv[4]
    config_filepath = sys.argv[5]

    config = Config(config_filepath)

    simulator = Simulator(sensor_list_filepath, previous_smarthome_data_filepath, slam_data_filepath, config_filepath)
    simulator.run_test(current_smarthome_data_filepath)

if __name__ == "__main__":
    main()
