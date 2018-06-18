#*****************************************************************************#
#**
#**  CASAS Location Library
#**
#**    Brian L Thomas, 2010
#**
#** Tools by the Center for Advanced Studies in Adaptive Systems at
#**  the School of Electrical Engineering and Computer Science at
#**  Washington State University
#** 
#** Copyright Washington State University, 2014
#** Copyright Brian L. Thomas, 2014
#** 
#** All rights reserved
#** Modification, distribution, and sale of this work is prohibited without
#**  permission from Washington State University
#** 
#** If this code is used for public research, any resulting publications need
#** to cite work done by Brian L. Thomas at the Center for Advanced Study of 
#** Adaptive Systems (CASAS) at Washington State University.
#** 
#** Contact: Brian L. Thomas (brian.thomas@email.wsu.edu)
#** Contact: Diane J. Cook (cook@eecs.wsu.edu)
#*****************************************************************************#
import re

myHash = dict()
revHash = dict()

def openFile(filename):
    list = open(filename)
    for x in list:
        y = str(x).strip()
        if len(y) > 1 and y[0] != "#":
            vals = re.split('\s+', x)
            if len(vals) > 1:
                myHash[vals[1]] = vals[0]
                revHash[vals[0]] = vals[1]
    return

def getLocation(serial):
    value = "unknown"
    if serial in myHash:
        value = myHash[serial]
    return value

def getSerial(location):
    value = "unknown"
    if location in revHash:
        value = revHash[location]
    return value
