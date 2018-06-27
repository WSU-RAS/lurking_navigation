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

        self.printIt()

        self.currentKey = -1
        #disperse map
        self.disperse()
        self.printIt()


    def printIt(self):

        for i in range(self.height):
            p = ""
            for j in range(self.width):
                k = (i, j)
                v = self.oMap[k].val
                p += str(v)
                p += " "
                if v == 0: p += " "

            print(p)

        print("\n \n \n")


    def createMap(self, mapObj):
        
        self.width = mapObj.width
        self.height = mapObj.height
        data = mapObj.data
        mapDict = {}
        counter = 0

        #Build dict of point objects, *hopefully* making later parts easier. 
        for i in range(self.height):
            for j in range(self.width):
                mapDict[(i,j)] = point.Point(i, j, data[counter], False) #May need to adjust points, right now its a 0,0. 
                counter += 1

        # for k, v in mapDict.iteritems():
        #     print(v.x, v.y, k, v)

        return mapDict
        
    
    #Start dispersing the walls down.
    def disperse(self):

        #Go through each point
        for k, v in self.oMap.iteritems():

            self.currentKey = k
            if v.val > 1 and not v.mark:
                self.outerEdge()

            else:
                self.oMap[k].mark = True


    #Go around the outer edge and change the values.
    def outerEdge(self):
        
        thirdList = self.thirds(self.oMap[self.currentKey].val)

        for i in range(len(thirdList)):

            edgeLen = (i + 1) * 2 + 1 
            self.change("R", edgeLen, thirdList[i])
            self.change("D", edgeLen, thirdList[i])
            self.change("L", edgeLen, thirdList[i])
            self.change("U", edgeLen, thirdList[i])

           


    #Change the edge
    def change(self, direction, edgeLen, newVal):

        #Still need to change origin to be up one and left one. 
        #Need to check key before we go. 

        for i in range(edgeLen):

            val = self.oMap[self.currentKey].val

            if val == -1:
                self.oMap[self.currentKey].mark = True

            elif val > newVal:
                pass

            else:
                print("Before: " + str(self.oMap[self.currentKey].val) + " Key: " + str(self.currentKey))
                self.oMap[self.currentKey].val = newVal
                self.oMap[self.currentKey].mark = True
                print("After: " + self.oMap[self.currentKey].val)


            #Not accounting for corners, oops. Change key and check it. 
            self.moveKey(direction)


            

    def moveKey(direct):

        print("Current Key " + str(self.currentKey))

        if direction == "R":
            self.currentKey = (self.currentKey[0] + 1, self.currentKey[1] + 0)

        elif direction == "D":
            self.currentKey = (self.currentKey[0] + 0, self.currentKey[1] + -1)

        elif direction == "L":
            self.currentKey = (self.currentKey[0] + -1, self.currentKey[1] + 0)

        elif direction == "U":
            self.currentKey = (self.currentKey[0] + 0, self.currentKey[1] + 1)

        else:
            print("No valid direction.")

        print("New Key " + str(self.currentKey))



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




