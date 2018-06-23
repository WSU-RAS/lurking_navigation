
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

    def run_dynamic_test(self, sensor_data_filepath):
        """ test the lurking ai algorithm
        
        load up and run the lurking ai in a simulated environment.
        Evaluate on DTD.

        """
        # constants
        decay_time = 60 # decay time in seconds
        time_scale = 5000.0 # multiplier applied to simulated time

        # open data files
        sensor_data_file = open(sensor_data_filepath, "r")

        # load up the dynamic heatmap
        dynamic_heatmap = DynamicHeatmap(self.sensor_map_filepath, self.config)

        # run the simulator until we run out of data
        distances = []
        previous_decay_time = None
        chosen_point = (48, 45)

        previous_timestamp = None
        distance = 99999

        while distance is not None: # step returns None if there is no more data
            # choose a point
            # TODO get real point. For now just run the heatmap
            chosen_point = (48, 45)

            # run dynamic step
            distance, path, point, data = self.temp_step(chosen_point, sensor_data_file)
            
            if distance < 9000: # ridiculously large distance means we cannot reach the point from our other point
                distances.append(distance)
                sensor_name = data.sensor_name
                time = data.timestamp

                # run decay times on the heatmap
                if previous_decay_time != None:
                    if time - previous_decay_time > decay_time:
                        decay_times_to_run = int((time - previous_decay_time) / decay_time)

                        # decay the map as many times as the decay time has passed
                        for i in xrange(decay_times_to_run):
                            dynamic_heatmap.decay_heatmap()

                        # update the previous time
                        previous_decay_time += decay_times_to_run * decay_time
                else:
                    previous_decay_time = time

                # print the time value
                print data.date + " " + data.time

                # update the heatmap values
                dynamic_heatmap.update_heatmap(sensor_name)

                # display the heatmap
                np_heatmap = np.array(dynamic_heatmap.heatmap)
                plt.imshow(np.transpose(np_heatmap), cmap='hot', interpolation='nearest')

                # display the slam map
                plt.imshow(np.transpose(self.slam_map.map), cmap=get_custom_colormap_green(), interpolation='nearest')

                # display the triggered point
                plt.plot(point[0], point[1], 'bo')

                # display functionality for the plot
                axis = plt.gca()
                if axis.get_ylim()[0] > axis.get_ylim()[1]:
                    axis.set_ylim(axis.get_ylim()[::-1])
                
                if previous_timestamp is not None:
                    pause_time = (time - previous_timestamp) / time_scale
                    pause_time = max(0.000001, pause_time) # ensure pause time does not evaluate to zero
                    pause_time = min(2.0, pause_time) # cap max pause time for differences between days.
                    print pause_time
                    plt.pause(pause_time)
                else:
                    plt.pause(0.000001)

                previous_timestamp = time
                plt.gcf().clear()


    def run_static_test(self, previous_sensor_data_filepath, current_sensor_data_filepath):
        # open data files
        previous_data_file = open(previous_sensor_data_filepath, "r")
        current_data_file = open(current_sensor_data_filepath, "r")

        # load up the base placer
        base_placer = BasePlacer(self.slam_map_filepath, self.sensor_map_filepath, previous_sensor_data_filepath, self.config)

        chosen_point = base_placer.get_best_point()

        distances = []
        distance = self.step(chosen_point, current_data_file)

        while distance is not None:
            if distance < 9000:
                distances.append(distance)
            distance = self.step(chosen_point, current_data_file)

        np_distances = np.array(distances)
        average = np.average(np_distances)
        maximum_distance = np.amax(np_distances)

        print "average distance: ", average
        print "maximum distance: ", maximum_distance
        plt.show()

    def temp_step(self, chosen_point, data_file):
        point, data = self._get_next_point(data_file, self.sensor_map, self.ignored_sensors)

        if point is None:
            return None, None, None, None

        # apply offset
        point_x = int(point[0] / self.config.map_resolution + self.config.slam_origin[0])
        point_y = int(point[1] / self.config.map_resolution + self.config.slam_origin[1])

        point = (point_x, point_y)

        # get distance
        distance, path = self.shortest_paths.run_shortest_paths(point, chosen_point)
        
        # check we actually got a distance.
        if distance is None:
            return 999999.9, None, point, data

        distance = distance * self.config.map_resolution

        return distance, path, point, data

    def step(self, chosen_point, data_file):
        """ run a single step of the simulation

        in this case get the next sensor trigger and tell how far it is from the given map value

        """

        point, time, date = self._get_next_point(data_file, self.sensor_map, self.ignored_sensors)

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

        plt.pause(0.00001)
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
                return point, data

def main():
    slam_data_filepath = sys.argv[1]
    sensor_list_filepath = sys.argv[2]
    previous_smarthome_data_filepath = sys.argv[3]
    current_smarthome_data_filepath = sys.argv[4]
    config_filepath = sys.argv[5]

    config = Config(config_filepath)

    simulator = Simulator(sensor_list_filepath, slam_data_filepath, config_filepath)
    #simulator.run_static_test(previous_smarthome_data_filepath, current_smarthome_data_filepath)
    simulator.run_dynamic_test(previous_smarthome_data_filepath)

if __name__ == "__main__":
    main()
