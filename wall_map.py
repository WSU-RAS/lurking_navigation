#!/usr/bin/env python


'''
Goal of this is to waterfall out all 
the edges. May need to be adjusted for 
what our needs are. 
'''


import waterfall, numpy
from slam_map import RawSlamData


class WallMap():

    def __init__(self):

        mapFile = "map_map.txt" #test file
        slamMap = RawSlamData(mapFile)
        self.wf = waterfall.Waterfall(slamMap)

    def getMap(self):
        return self.wf.toNumpy()



#Testing
if __name__ == '__main__':

    wm = WallMap()
    newMap = wm.getMap()
    print(newMap)