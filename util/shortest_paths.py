

import itertools


from scipy.sparse.dok import dok_matrix
from scipy.sparse.csgraph import dijkstra


class ShortestPaths:
    """

        class for getting the shortest path
        between any two points in a numpy array with
        a threshold of what is a barrier or not

    """

    def __init__(self, map):
        """ init shortest paths

        input:
            map - a 2d numpy array representing all reachable areas
            of the map. (probably just the reachability map)
        """

        self.map = map

        # init the adjacency matrix
        self.adjacency = dok_matrix((map.shape[0] * map.shape[1],
                                     map.shape[0] * map.shape[1]), dtype=bool)

        # fill the adjacency matrix
        self._fill_adjacency()

    def _fill_adjacency(self):
        directions = list(itertools.product([0, 1, -1], [0, 1, -1]))
        _map = self.map

        for i in range(0, _map.shape[0]):
            for j in range(0, _map.shape[1]):
                if not _map[i, j]:
                    continue

                if _map[i, j] == 0:
                    continue

                for x_diff, y_diff in directions:
                    if _map[i + x_diff, j + y_diff] > 0:
                        start_i = self.to_index(i, j)
                        end_i = self.to_index(i + x_diff, j + y_diff)
                        self.adjacency[start_i, end_i] = True

    def run_shortest_paths(self, start_point, end_point):
        """ run shortest paths on the two points in our graph

        run the shortest path algorithm and return the distance between
        the two points.

        """
        if self.check_bounds(start_point) is False or \
           self.check_bounds(end_point) is False:
            return None, None

        start = self.point_to_index(start_point)
        end = self.point_to_index(end_point)

        distances, predecessors = dijkstra(self.adjacency, directed=False,
                                           indices=start, unweighted=True,
                                           return_predecessors=True)

        distance = distances[end]
        if distance < 0 or distance > self.map.shape[0] * self.map.shape[1]:
            return None, None

        path = self._get_path(predecessors, start_point, end_point)

        return distance, path

    def _get_path(self, predecessors, start_point, end_point):
        """ get the shortest path from start to end
        """
        path_list = []

        start_i = self.point_to_index(start_point)

        current = self.point_to_index(end_point)
        while current != start_i:
            path_list.append(self.to_coordinates(current))
            current = predecessors[current]

        path_list = reversed(path_list)

        return path_list

    def check_bounds(self, point):
        if point[0] < 0 or point[1] < 0:
            return False

        if point[0] > self.map.shape[0] or point[1] > self.map.shape[1]:
            return False

    def point_to_index(self, point):
        """ convert a point tuple to an index in our flat array
        """
        return self.to_index(point[0], point[1])

    def to_index(self, x, y):
        """ convert a set of coordinates to an index in our flat array
        """
        return x + y * self.map.shape[1]

    def to_coordinates(self, index):
        return index / self.map.shape[1], index % self.map.shape[1]




