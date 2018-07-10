"""

methods for transforming points between the various coordinate systems that we
use.

"""

import math


def rotate_point(point, origin_point, degrees):
    """ rotate a point around another point

    code courtesy of Mark Dickinson on Stack overflow.
    Link: https://stackoverflow.com/questions/34372480/rotate-point-about-another-point-in-degrees-python 

    rotate a point clockwise by the given angle around the given origin.

    input:
        point - tuple(float(x), float(y))- point to rotate
        origin_point -tuple(float(x), float(y)) - point to rotate around
        degrees - float - number of degrees to rotatate

    returns:
        
    """
    ox, oy = origin_point
    px, py = point

    # invert so we are going clockwise
    degrees = -degrees

    # convert degrees to radians
    rads = math.radians(degrees)

    # perform the rotation
    qx = ox + math.cos(rads) * (px - ox) - math.sin(rads) * (py - oy)
    qy = oy + math.sin(rads) * (px - ox) - math.cos(rads) * (py - oy)

    return qx, qy

def transform_back_to_slam(point, slam_map):
    """ transform a point back to slam space

    Takes a point and rotates it, then finds it's difference from the slam map
    origin, converts that to meters, and outputs.

    input:
        point - Tuple(float(x), float(y)) - point to transform
        slam_map - SlamMap - slam map to transform back through
    """

    # perform rotation
    center = (slam_map.map.shape[0] / 2, slam_map.map.shape[1] / 2)
    point = rotate_point(point, center, slam_map.config.slam_rotation)

    # find difference with respect to the origin
    diff = tuple(i - j for i, j in zip(point, slam_map.origin))

    # convert to meters
    diff = tuple(i * slam_map.config.map_resolution for i in diff)

    return diff
