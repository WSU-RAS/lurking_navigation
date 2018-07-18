#!/usr/bin/env python

import sys
import math
import numpy as np

import matplotlib.pyplot as plt

from heatmap import Heatmap
from dynamic_heatmap import DynamicHeatmap
from config import Config

class PathMap:
    """ represents a map of estimated paths that the person takes through the house
    
    Takes in a heatmap and uses the heatmap to create a undirected graph. 
    The weight between two adjacent nodes is the average of their two weights.

    Note that if you adjust the offset of the heatmap after initializing this PathMap it will no longer line up

    """

    def __init__(self, heatmap, maximum_distance=2.5, path_thickness=2.0):
        """ build the path map from the heatmap

        inputs:
            heatmap - Heatmap - heatmap used to build the path map
            maximum_distance - float - maximum distance to be considered a neighboring point (in meters)
            path_thickness - float - thickness of path to lay down between points (in meters)
        """
        # variable declarations
        self.nodes = {}
        self.maximum_distance = maximum_distance
        self.path_thickness = path_thickness
        self.heatmap = heatmap

        # build graph
        self._build_graph()

    def get_as_array(self):
        """ get the path map as an array

        convert the internal graph representation to a 2d array and return it

        returns:
            2d numpy array representing our estimated traffic through each point

        """
        heatmap = self.heatmap
        a_width = int(math.ceil(heatmap.map_width / heatmap.map_resolution))
        a_height = int(math.ceil(heatmap.map_height / heatmap.map_resolution))

        # init empty array
        path_map = np.zeros((a_width, a_height))
        
        # load the lines into the array
        for point, node in self.nodes.iteritems():
            for edge in node.edges:
                resolution = self.heatmap.map_resolution

                p1 = node.point
                p2 = edge[0].point
                thickness = self.path_thickness / resolution
                weight = edge[1]

                # convert points to correct resolution
                p1 = (p1[0] / resolution, p1[1] / resolution)
                p2 = (p2[0] / resolution, p2[1] / resolution)

                # convert thickness to correc
                path_map = draw_weighted_line(path_map, p1, p2, thickness, weight)

        return path_map

    def display_as_heatmap(self):
        array = self.get_as_array()

        plt.imshow(np.transpose(array), cmap='hot', interpolation='nearest')

        axis = plt.gca()
        axis.set_ylim(axis.get_ylim()[::-1])
        plt.show()

    def _build_graph(self):
        point_map = self.heatmap.get_point_map() 

        # build each point
        for point, value in point_map.iteritems():
            node = PathNode(value, point)
            self.nodes[point] = node

        # iterate through each node
        for point, _ in point_map.iteritems():
            node = self.nodes[point]

            # find neighbors
            for other_point, _ in point_map.iteritems():
                distance = get_point_distance(point, other_point)
                if distance < self.maximum_distance:
                    # add to list of edges
                    other_node = self.nodes[other_point]
                    node.add_edge(other_node)

class PathNode:
    """ represents a single sensor in the house
    """
    def __init__(self, weight=0.0, point=None):
        self.edges = [] # (node, weight)
        self.weight = weight
        self.point = point

    def add_edge(self, other_node):
        """ add edge to this node

        add a weighted edge to this node connecting it to another node.
        Note that the edge weight is calculated automoatically.

        inputs:
            other_node - other node to connect this node to.
        """
        # calculate edge weight
        weight = (self.weight + other_node.weight) / 2
        edge = (other_node, weight)

        # actually add the edge
        assert edge not in self.edges
        self.edges.append(edge)

def draw_weighted_line(array, start_point, end_point, thickness, weight):
    rmax = array.shape[1]

    # get the mask we need to apply
    mask = weighted_line(start_point[0], start_point[1], end_point[0], end_point[1], thickness, rmin=0, rmax=rmax)

    # apply the mask with the given weight
    # note that we take the maximum of this new value and any value that might already be there.
    x_mask = mask[0]
    y_mask = mask[1]
    vals = mask[2]

    array_w_line = np.copy(array)
    array_w_line[x_mask, y_mask] = vals * weight

    array = np.maximum(array, array_w_line)
    return array

def trapez(y,y0,w):
    return np.clip(np.minimum(y+1+w/2-y0, -y+1+w/2+y0),0,1)

def weighted_line(r0, c0, r1, c1, w, rmin=0, rmax=np.inf):
    """ get mask for drawing a weighted line on a 2d array

    Courtesy of Marco Spinaci on Stack Overflow
    link: https://stackoverflow.com/a/47381058
    """
    # The algorithm below works fine if c1 >= c0 and c1-c0 >= abs(r1-r0).
    # If either of these cases are violated, do some switches.
    if abs(c1-c0) < abs(r1-r0):
        # Switch x and y, and switch again when returning.
        xx, yy, val = weighted_line(c0, r0, c1, r1, w, rmin=rmin, rmax=rmax)
        return (yy, xx, val)

    # At this point we know that the distance in columns (x) is greater
    # than that in rows (y). Possibly one more switch if c0 > c1.
    if c0 > c1:
        return weighted_line(r1, c1, r0, c0, w, rmin=rmin, rmax=rmax)

    # The following is now always < 1 in abs
    if math.fabs(c1-c0) < 0.005:
        slope = 999999999.9
    else:
        slope = (r1-r0) / (c1-c0)

    # Adjust weight by the slope
    w *= np.sqrt(1+np.abs(slope)) / 2

    # We write y as a function of x, because the slope is always <= 1
    # (in absolute value)
    x = np.arange(c0, c1+1, dtype=float)

    if math.fabs(c1-c0) < 0.005:
        y = x * slope + 99999999999.9
    else:
        y = x * slope + (c1*r0-c0*r1) / (c1-c0)

    # Now instead of 2 values for y, we have 2*np.ceil(w/2).
    # All values are 1 except the upmost and bottommost.
    thickness = np.ceil(w/2)
    yy = (np.floor(y).reshape(-1,1) + np.arange(-thickness-1,thickness+2).reshape(1,-1))
    xx = np.repeat(x, yy.shape[1])
    vals = trapez(yy, y.reshape(-1,1), w).flatten()

    yy = yy.flatten()

    # Exclude useless parts and those outside of the interval
    # to avoid parts outside of the picture
    mask = np.logical_and(yy >= rmin, yy < rmax, vals > 0)

    return (yy[mask].astype(int), xx[mask].astype(int), vals[mask])

def get_point_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def main():
    sensor_filepath = sys.argv[1]
    config_filepath = sys.argv[2]
    config = Config(config_filepath)

    # static_heatmap = StaticHeatmap(sensor_filepath, config)

    pathmap = PathMap(static_heatmap)
    pathmap.display_as_heatmap()

if __name__ == "__main__":
    main()
