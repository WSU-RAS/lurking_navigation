
import sys

import numpy as np
import matplotlib.pyplot as plt

# maps and lower level stuff
from config import Config
from heatmap import get_sensor_map, DataLine
from slam_map import SlamMap
from path_map import get_point_distance
from reachability_map import ReachabilityMap

# util
from util.shortest_paths import ShortestPaths
from overlay import get_custom_colormap_green

# algorithms to evaluate
from base_placement_algorithm import BasePlacer
from dynamic_heatmap import DynamicHeatmap


class Simulator:
    """ Simulator environment for playing back sensor data

    We want to take in a file of sensor triggers and play it back to both 
    run our real time algorithms and evaluate algorithms against battery life
    and DTD (Distance to delivery)

    """
    def __init__(self, sensor_map_filepath, slam_map_filepath, config_filepath):
        # setup low level variables (needed to init the tests)
        self.sensor_map_filepath = sensor_map_filepath
        self.slam_map_filepath = slam_map_filepath
        self.config_filepath = config_filepath

        # setup high level variables
        self.config = Config(config_filepath)
        self.sensor_map, self.ignored_sensors = get_sensor_map(sensor_map_filepath)
        self.slam_map = SlamMap(slam_map_filepath, self.config)
        self.reachability_map = ReachabilityMap(self.slam_map)

        self.shortest_paths = ShortestPaths(self.reachability_map.map)

    def run_static_test(self, 
                        previous_sensor_data_filepath,
                        current_sensor_data_filepath):
        """ test the static base placement algorithm
        
        get the best point from the base placement algorithm from a previous
        data file and run it against a more recent data file to check how
        well it does in terms of distance.

        input:
            previous_sensor_data_filepath - filepath to data file to use as
                the prior data to initialize the base placement algorithm.
            current_sensor_data_filepath - filepath to data file to use as
                the data to test against

        returns:
            average distance - average distance of the chosen point of the
                base placer to the actual position of the people in the
                data file
            maximum distance - maximum distance of the chosen point of the
                base placer to the actual position of the people in the data
                file.

        """
        # open data files
        current_data_file = open(current_sensor_data_filepath, "r")

        # load up the base placer
        base_placer = BasePlacer(self.slam_map_filepath,
                                 self.sensor_map_filepath,
                                 previous_sensor_data_filepath,
                                 self.config)
        chosen_point = base_placer.get_best_point()

        # run simulation until we have  no other data
        distances = []
        flag, results = self.step(chosen_point, current_data_file)

        while flag != "no_more_data":
            if flag == "all_good":
                distances.append(results['distance'])

                # display the simulator step
                self.display_simulator_step(chosen_point,
                                            results['point'],
                                            results['path'])

            flag, results = self.step(chosen_point, current_data_file)

        # extract and display statistics
        average_distance = np.average(distances)
        maximum_distance = np.amax(distances)

        print "average distance: ", average_distance
        print "maximum distance: ", maximum_distance

        return average_distance, maximum_distance

    def step(self, chosen_point, data_file):
        """ run a single step of the simulation

        get the next sensor trigger from the file and check how far it is from
        the chosen point.

        Note that there are a few cases that can happen in any simulator step:
            all good - no problems. results is the full tuple
            no more data - there is no more data available in the data file.
                           results will be None.
            path not found - shortest paths could not find a path between the
                             two values

        inputs:
            chosen_point - point chosen by the algorithm to be evaluated
            data_file - file of sensor triggers

        returns:
            flag - string - flag marking the case we returned out of
                            any of: ('path_not_found', 'no_more_data', 'all_good')
            results - dictionary (distance, path, point, data) - results of the time step
                distance - float - distance in meters needed for the chosen point to
                                   get to the next point from the data file
                path - list(points) - list of points to get from the chosen point to
                                      the next point from the data file.
                data - DataLine - data from the relevant line of the data file.

        """

        results = {}

        # get the next batch of information from the data file
        point, data = self._get_next_point(data_file, self.sensor_map, self.ignored_sensors)

        if point is None:
            # data file has no more points
            return 'no_more_data', None

        # apply offset
        point_x = int(point[0] / self.config.map_resolution + self.config.slam_origin[0])
        point_y = int(point[1] / self.config.map_resolution + self.config.slam_origin[1])

        point = (point_x, point_y)

        # get distance and path
        distance, path = self.shortest_paths.run_shortest_paths(point, chosen_point)

        # check we actually got a distance.
        if distance is None:
            # could not find a path between the chosen point and point
            results['data'] = data
            return 'path_not_found', results

        distance = distance * self.config.map_resolution

        results['distance'] = distance
        results['path'] = path
        results['point'] = point
        results['data'] = data
        return 'all_good', results

    def display_simulator_step(self, chosen_point, next_point, path):
        # display the points and slam map
        plt.imshow(np.transpose(self.slam_map.map), cmap='hot', interpolation='nearest')

        point = next_point

        x, y = zip(*path)
        plt.scatter(y, x)

        plt.plot(point[0], point[1], 'bo')
        plt.plot(chosen_point[0], chosen_point[1], 'go')

        axis = plt.gca()
        if axis.get_ylim()[0] > axis.get_ylim()[1]:
            axis.set_ylim(axis.get_ylim()[::-1])

        plt.pause(0.00001)
        plt.gcf().clear()

    def _get_next_point(self, smarthome_data_file, sensor_map, ignored_sensors):
        while True:
            next_line = smarthome_data_file.readline()

            # check if the file is done
            if len(next_line) == 0:
                return None, None

            data = DataLine(next_line)

            if data.sensor_type == "Control4-Motion" \
               and data.message == "ON" \
               and data.sensor_name not in ignored_sensors:
                point = sensor_map[data.sensor_name]
                return point, data

def main():
    slam_data_filepath = sys.argv[1]
    sensor_list_filepath = sys.argv[2]
    previous_smarthome_data_filepath = sys.argv[3]
    current_smarthome_data_filepath = sys.argv[4]
    config_filepath = sys.argv[5]

    config = Config(config_filepath)

    simulator = Simulator(sensor_list_filepath, slam_data_filepath, config_filepath)
    simulator.run_static_test(previous_smarthome_data_filepath, current_smarthome_data_filepath)
    #simulator.run_dynamic_test(previous_smarthome_data_filepath)

if __name__ == "__main__":
    main()
