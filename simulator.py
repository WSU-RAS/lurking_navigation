
import sys

import numpy as np
import matplotlib.pyplot as plt

# maps and lower level stuff
from config import Config
from heatmap import get_sensor_map, DataLine
from slam_map import SlamMap
from reachability_map import ReachabilityMap

# util
from util.shortest_paths import ShortestPaths
from overlay import get_custom_colormap_green
from lurking_ai import TIME_TICK

# algorithms to evaluate
from base_placement_algorithm import BasePlacer
from lurking_ai import LurkingAI
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

    def run_dynamic_test(self, previous_sensor_data_filepath,
            current_sensor_data_filepath, display=True):
        """ test the real time lurking algorithm

        get the best point recommended by the algorithm at every timestep
        from the sensor data file and check how it does at every step.

        input:
            sensor data filepath - filepath to data to test against.

        returns:
            average distance - average distance of the point chosen by the
                lurking algorithm to the actual position given by the data file
            maximum distance - maximum distance from the point chosen by the
                lurking algorithm to the actual position given by the data file

        """

        # constants
        decay_time = 60.0 # time between decay points in seconds
        time_scale = 5000.0 # multiplier appplied to simulated time
        min_time_delay = 0.000001
        max_time_delay = 2.0
        start_point = (48, 45)
        print_delay = 100

        # open data files
        previous_sensor_data_file = open(previous_sensor_data_filepath, "r")
        current_sensor_data_file = open(current_sensor_data_filepath, "r")

        # start lurking AI in simulator mode
        dynamic_heatmap = DynamicHeatmap(self.sensor_map_filepath, self.config)
        self.lurking_ai = LurkingAI(dynamic_heatmap, self.slam_map_filepath,
                               self.sensor_map_filepath, 
                               self.config, simulated=True, display_all=True)
        self.decay_time = TIME_TICK

        # init variables
        distances = []
        self.previous_decay_time = None
        self.previous_timestamp = None
        self.current_time = None
        self.num_steps = 0
        chosen_point = start_point # hardcode for now

        # load the lurking ai with previous data
        print "running previous data"
        self.dynamic_run_previous_data(previous_sensor_data_file)
        print "finished running previous data"

        # run the simulator until we run out of data
        flag, results = self.step(chosen_point, current_sensor_data_file)

        while flag != 'no_more_data':
            # choose a point
            chosen_point = self.lurking_ai.get_landing_zone()

            if flag == 'all_good':
                distances.append(results['distance'])

                self.current_time = results['data'].timestamp

                # run decay times on the heatmap
                self.run_decay_times()

                # update the lurker
                self.lurking_ai.update_heatmap(results['data'])

                # calculate time delay
                if display is True:
                    if self.previous_timestamp is not None:
                        time_delay = (self.current_time - self.previous_timestamp) / time_scale
                        time_delay = max(min_time_delay, time_delay) # ensure time delay
                            # does not evaluate to zero
                        time_delay = min(max_time_delay, time_delay) # cap max time
                            # delay for when we move to the next day
                    else:
                        time_delay = min_time_delay


                    # display
                    self.display_simulator_step(chosen_point, results['point'],
                                                results['path'], time_delay)

                self.previous_timestamp = self.current_time

                if self.num_steps % print_delay == 0:
                    print "finished step: ", self.num_steps

            elif flag == 'path_not_found':
                self.lurking_ai.update_heatmap(results['data'])
            else:
                raise Exception("invalid flag")


            # run a step
            flag, results = self.step(chosen_point, current_sensor_data_file)
            self.num_steps += 1

        # extract and display statistics
        average_distance = np.average(distances)
        maximum_distance = np.amax(distances)

        print "average distance: ", average_distance
        print "maximum distance: ", maximum_distance

        return average_distance, maximum_distance
    
    def dynamic_run_previous_data(self, previous_sensor_data_file):
        chosen_point = (0, 0)
        flag = 'None'
        self.num_steps = 0

        flag, results = self.step(chosen_point, previous_sensor_data_file)

        while flag != 'no_more_data':

            # run decay times
            self.current_time = results['data'].timestamp
            self.run_decay_times()

            # update the lurker
            self.lurking_ai.update_heatmap(results['data'])

            if self.num_steps % 100 == 0:
                print "finished step: ", self.num_steps

            self.num_steps += 1
            flag, results = self.step(chosen_point, previous_sensor_data_file)

        # reset number of steps
        self.num_steps = 0

    def run_decay_times(self):
        if self.previous_decay_time != None:
            if self.current_time - self.previous_decay_time > self.decay_time:
                decay_times_to_run = int(
                        (self.current_time - self.previous_decay_time) / self.decay_time)

                # decay as many times as necessary 
                for _ in xrange(decay_times_to_run):
                    self.lurking_ai.timer_callback()

                # update previous decay time
                self.previous_decay_time += decay_times_to_run * self.decay_time
        else:
            previous_decay_time = self.current_time


    def run_static_test(self,
                        previous_sensor_data_filepath,
                        current_sensor_data_filepath,
                        display=True):
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
        num_steps = 0

        flag, results = self.step(chosen_point, current_data_file)

        while flag != "no_more_data":
            if flag == "all_good":
                distances.append(results['distance'])

                # display the simulator step
                if display is True:
                    self.display_simulator_step(chosen_point,
                                                results['point'],
                                                results['path'])

            flag, results = self.step(chosen_point, current_data_file)
            if num_steps % 100 == 0:
                print "finished step: ", num_steps

            num_steps += 1

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

    def display_simulator_step(self, chosen_point, next_point, path,
                               time_delay=0.000001):
        """ display a single step of the simulator

        displays the slam map and overlays the chosen point, next point,
        and the path between the two.

        input:
            chosen_point - tuple(int(x), int(y)) - point chosen by the algorithm
            next_point - tuple(int(x), int(y)) - point given by the data file
            path - list(tuple(int(x), int(y))) - list of points that make up
                the path between the chosen point and the next point.
            time_delay - float - time delay in seconds to hang before returning

        returns: None
        """
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

        plt.pause(time_delay)
        plt.gcf().clear()

    def _get_next_point(self, smarthome_data_file, sensor_map, ignored_sensors):
        """ get the next valid point from the data file

        run through the data file until we find a sensor trigger that is the
        right type and is not in the list of ignored sensors.

        inputs:
            smarthome_data_file - file - data file to look through
            sensor_map - dictionary{string(sensor_name):point(location)} -
                map of sensor locations
            ignored_sensors - list(string(sensor_name)) - list of sensors that
                we do not have locations for.

        returns:
            point - tuple(float(x), float(y)) - location of the next sensor
                triggered.
            data - DataLine - data container of information about the trigger
        """
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

    simulator = Simulator(sensor_list_filepath, slam_data_filepath, config_filepath)
    simulator.run_static_test(previous_smarthome_data_filepath, current_smarthome_data_filepath, display=False)
    #simulator.run_dynamic_test(previous_smarthome_data_filepath, current_smarthome_data_filepath, display=False)

if __name__ == "__main__":
    main()
