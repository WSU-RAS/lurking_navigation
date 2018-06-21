

import numpy as np

class WallMap:
    """

    Goal: map with high values by walls and especially high values near corners.

    propagate value away from walls according to the inverse square law.

    """
    def __init__(self, slam_map):
        # constants
        self.wall_value = 1.0
        self.stop_cutoff = 0.01
        
        # variables
        self.map = None

        self._build_map(slam_map)
    
    def _build_map(self, slam_map):
        s_map = slam_map.map
        wall_map = np.zeros_like(s_map, dtype=float)

        for i in range(s_map.shape[0]):
            for j in range(s_map.shape[1]):
                # for each point propagate out 
                pass
