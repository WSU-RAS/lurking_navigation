#!/usr/bin/env python


'''
Waterfall logic
'''


import point


class Waterfall():

    def __init__(self, mapObj):
        
        #create map
        self.width = 0
        self.height = 0
        self.oMap = self.createMap(mapObj)

        #Testing purposes
        self.printIt()

        self.currentKey = -1 #Keep track of key here, makes it much easier than passing and returning keys all ove rthe place. 

        #disperse map
        self.disperse()
        
        #Testing purposes
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
        counter = 0
        for k, v in self.oMap.iteritems():
            
            self.currentKey = k
            if v.val > 1 and not v.mark:
                self.outerEdge()
                # self.printIt()
                counter += 1

            else:
                self.oMap[k].mark = True

            if counter > 0:
                break


    #Go around the outer edge and change the values.
    def outerEdge(self):
        
        #Change origin to be in the next area. Such a if you're in a 3x3 square:
        # o - - 
        # - x -
        # - - -
        # and your current point is x you want to move to o before going around the edge. 
        # go right 3 times.
        # go down 2 times.
        # go left 2 times.
        # go up 1 time.

        thirdList = self.thirds(self.oMap[self.currentKey].val)
        self.oMap[self.currentKey].mark = True #We're changing it now
        # self.currentKey = (self.currentKey[0]-1, self.currentKey[1]+1) #Up one and left one.
        dirs = ["R", "D", "L", "U"]

        for letter in dirs:
            for i in range(len(thirdList)):
                edgeLen = (i + 1) * 2 + 1
                originalKey = self.currentKey

                if letter == "D" and i < len(thirdList) - 1:
                    print("going down for the " + str(i) + "th time.")
                    self.change(letter, edgeLen, thirdList[i])  

                elif letter == "L" and i < len(thirdList) - 1:
                    print("going left for the " + str(i) + "th time.")
                    self.change(letter, edgeLen, thirdList[i])  

                elif letter == "U" and i < len(thirdList) - 2:
                    print("going up for the " + str(i) + "th time.")
                    self.change(letter, edgeLen, thirdList[i])

                elif letter == "R": 
                    print("going right for the " + str(i) + "th time.")
                    self.change(letter, edgeLen, thirdList[i])

        self.currentKey = originalKey


        # for i in range(len(thirdList)):

        #     edgeLen = (i + 1) * 2 + 1 

        #     originalKey = self.currentKey

        #     for letter in dirs:
        #         print (i)

        #         if letter == "D" and i < len(thirdList) - 1:
        #             print("going down for the " + str(i) + "th time.")
        #             self.change(letter, edgeLen, thirdList[i])  

        #         elif letter == "L" and i < len(thirdList) - 1:
        #             print("going left for the " + str(i) + "th time.")
        #             self.change(letter, edgeLen, thirdList[i])  

        #         elif letter == "U" and i < len(thirdList) - 2:
        #             print("going up for the " + str(i) + "th time.")
        #             self.change(letter, edgeLen, thirdList[i])

        #         else: 
        #             print("going right for the " + str(i) + "th time.")
        #             self.change(letter, edgeLen, thirdList[i])

        #     self.currentKey = originalKey


    #Change the edge
    def change(self, direction, edgeLen, newVal):

        #Need to check key before we go. 

        # for i in range(edgeLen):
            # self.printIt()
            # print(self.currentKey)
        if self.currentKey not in self.oMap.keys():
            print("Bad key...")
            self.moveKey(direction) #Move key because its borked

        else:
            print("Good key...")
            val = self.oMap[self.currentKey].val
            if val == -1:
                self.oMap[self.currentKey].mark = True

            elif val > newVal:
                pass

            else:
                self.oMap[self.currentKey].val = newVal
                self.oMap[self.currentKey].mark = True

            #Move the key after
            print("Before " + str(self.currentKey))
            self.moveKey(direction)
            print("After " + str(self.currentKey))


    #Move the key pointer.
    def moveKey(self, direction):

        # print("Current Key " + str(self.currentKey))

        if direction == "R":
            self.currentKey = (self.currentKey[0] + 1, self.currentKey[1])

        elif direction == "D":
            self.currentKey = (self.currentKey[0], self.currentKey[1] - 1)

        elif direction == "L":
            self.currentKey = (self.currentKey[0] - 1, self.currentKey[1])

        elif direction == "U":
            self.currentKey = (self.currentKey[0], self.currentKey[1] + 1)

        else:
            print("No valid direction.")

        # print("New Key " + str(self.currentKey))



    #Get all the thirds of a value
    def thirds(self, val):

        done = False
        thirds = []
        
        while not done:
            
            val = round(val/3.0)
            thirds.append(int(val))

            if val <= 1:
                done = True

        return thirds




