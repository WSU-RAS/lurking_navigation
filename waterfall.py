#!/usr/bin/env python


'''
Waterfall logic. Given a SLAM map obj you can give it to
the waterfall and it creates a waterfall effect for each
point. If there is conflicting points it picks the larger
of the two points. 
'''


import point, numpy


class Waterfall():

    def __init__(self, mapObj):
        
        self.width = 0
        self.height = 0
        self.oMap = self.createMap(mapObj) #Create the map
        self.currentKey = -1 #Keep track of key here, makes it much easier than passing and returning keys all over the place. 
        self.printIt()
        self.disperse() #Waterfall it out
        self.printIt()


    #Print the map, this is for testing, once that's done it won't be called any longer. 
    #It will remain if more debugging needs to be done in the future. 
    def printIt(self):

        for i in range(self.height):
            p = ""
            for j in range(self.width):
                k = (j, i)
                v = self.oMap[k].val
                p += str(v) #Build the string
                p += " "
                if v == 0: p += " " #Extra space around 0 or it looks weird

            print(p) #Print it

        print("\n")  #Extra line


    #Convert the dict to 2D array
    def toNumpy(self):

        np_map = numpy.zeros([self.height, self.width], dtype=int) #2D array with the heigth/width for row/cols

        for k, v in self.oMap.iteritems(): #Move each item in the dict into the map.
            
            x = k[0]
            y = k[1]
            val = v.val
            np_map[y, x] = val

        return np_map


    #Create the map from the raw text from the slam_map.py
    def createMap(self, mapObj):
        
        self.width = mapObj.width
        self.height = mapObj.height
        data = mapObj.data
        mapDict = {}
        counter = 0

        #Build dict of point objects, *hopefully* making later parts easier. 
        #Each point includes x, y, value and if it's been marked. 
        #Right now there's no point to the x and y because it's built with a 
        #0,0 index. However this might change. 
        for i in range(self.height):
            for j in range(self.width):
                mapDict[(j,i)] = point.Point(j, i, data[counter], False) #May need to adjust points, right now its a 0,0. 
                counter += 1

        return mapDict
        
    
    #Start dispersing the walls down.
    def disperse(self):

        #Go through each point
        for k, v in self.oMap.iteritems():
            
            self.currentKey = k #Set current key
            if v.val >= 10 and not v.mark: #The point is greater or equal to 10 and hasn't been touched. Less than 10 is pointless to waterfall. 
                self.outerEdge() #Go waterfall it
                self.oMap[k].mark = True #Make sure the point we just did is marked as true

            else:
                self.oMap[k].mark = True #Even if it isn't, mark it as being done. 



    #Go around the outer edge and change the values.
    def outerEdge(self):

        #Change origin to be in the next area. Such a if you're in a 3x3 square:
        # o - - 
        # - x -
        # - - -
        # and your current point is x you want to move to o before going around the edge. 
        # go right 2 times.
        # go down 2 times.
        # go left 2 times.
        # go up 2 times to get back to where we were.

        thirdList = self.thirds(self.oMap[self.currentKey].val) #Get all the third values of current value.
        self.oMap[self.currentKey].mark = True #Marking ourselves as done. 
        dirs = ["R", "D", "L", "U"]

        for j in range(len(thirdList)): #For each of the items we need to go do around and change values.
            self.currentKey = (self.currentKey[0]-1, self.currentKey[1]-1) #Up one and left one, 0,0 is top left.
            edgeLen = (j + 1) * 2 #Lenght of the edge. 

            for dir in dirs: #Go each direction, IN THAT ORDER, that part is very important. 
                for i in range(edgeLen): #Go that direction edge times. 
                    self.change(dir, thirdList[j]) #Change item and move 1 to the dir. 


    #Change the edge
    def change(self, direction, newVal):

        if self.currentKey not in self.oMap.keys(): #The key isn't in the dict. This is ok, it just means it's out of range and we need to move around edges/corners. 
            self.moveKey(direction) #Move key because its borked

        else: #It's a good key

            val = self.oMap[self.currentKey].val #current value
            
            if val == -1:  #Val is no-no zone. 
                self.oMap[self.currentKey].mark = True

            elif val > newVal: #If the current val is less than the new val, ignore it. 
                pass

            else: #Nothing else
                self.oMap[self.currentKey].val = newVal #Change val
                self.oMap[self.currentKey].mark = True #Mark it as changed, don't waterfall it out. 

            #Move the key after
            self.moveKey(direction) #Move it. 


    #Move the key pointer.
    def moveKey(self, direction):
        
        #These are based on having (0, 0) in the top left corner. 
        if direction == "R": self.currentKey = (self.currentKey[0] + 1, self.currentKey[1]) #(1, 0)

        elif direction == "D": self.currentKey = (self.currentKey[0], self.currentKey[1] + 1) #(0, 1)

        elif direction == "L": self.currentKey = (self.currentKey[0] - 1, self.currentKey[1]) #(-1, 0)

        elif direction == "U": self.currentKey = (self.currentKey[0], self.currentKey[1] - 1) #(0, -1)

        else: 
            print("No valid direction.") #This should NOT happen.


    #Get all the thirds of a value
    def thirds(self, val):

        done = False
        thirds = []
        
        while not done:
            
            val = round(val/3.0)
            if val >= 10: thirds.append(int(val))
            elif val < 10: done = True

        return thirds