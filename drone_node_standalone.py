#!/usr/bin/env python

PKG = 'drone'
import roslib; roslib.load_manifest(PKG)
import rospy
import sys
import math
import numpy
import random
import time
import json
from drone.msg import GPS_Coord
from drone.msg import GasSensorData

# Drone Class. Maintains ID, exploration flag, GPS location, time, objective GPS location, exploration grid bounds, 
# resolution of exploration grid, data aggregator object, exploration mapping object.
class Drone:
    # Constructor. Requires start GPS location, exploration area size (in feet), grid resolution, and id
    def __init__(self, x, y, z, xRangeDiameter, yRangeDiameter, zRangeHeight, resolutions, id):
        self.proximityReq = 1                       # Max distance to target to consider objective location reached
        self.isExploring = True
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.t = 0

        # Calculate explorable area grid bounds in terms of GPS locations based on input exploration area size
        xMax = x + (xRangeDiameter/2)*0.0003048/10000*90
        xMin = x - (xRangeDiameter/2)*0.0003048/10000*90
        yMax = y + (yRangeDiameter/2)*0.0003048/10000*90
        yMin = y - (yRangeDiameter/2)*0.0003048/10000*90
        zMax = z + (zRangeHeight/2)*0.0003048/10000*90
        zMin = z - (zRangeHeight/2)*0.0003048/10000*90
        bounds = {'xMax':xMax,'xMin':xMin,'yMax':yMax,'yMin':yMin,'zMin':zMin,'zMax':zMax}
        self.bounds = bounds

        self.resolutions = resolutions
        self.xResolutionSize = (bounds['xMax'] - bounds['xMin'])/self.resolutions['x']
        self.yResolutionSize = (bounds['yMax'] - bounds['yMin'])/self.resolutions['y']
        self.zResolutionSize = (bounds['zMax'] - bounds['zMin'])/self.resolutions['z']

	    # Initial start destination is furthest exploration grid corner
        if x < ((bounds['xMax'] - bounds['xMin']) / 2):
            self.xObjective = bounds['xMax']
        else:
            self.xObjective = bounds['xMin']
        if y < ((bounds['yMax'] - bounds['yMin']) / 2):
            self.yObjective = bounds['yMax']
        else:
            self.yObjective = bounds['yMin']
        if z < ((bounds['zMax'] - bounds['zMin']) / 2):
            self.zObjective = bounds['zMax']
        else:
            self.zObjective = bounds['zMin']
        self.dataCollection = DroneDataAggregator(self.x, self.y, self.z, self.xObjective, self.yObjective, self.zObjective, bounds, resolutions)
        self.maps = DroneObjectiveMapping( bounds, resolutions, id )

    # Update drone GPS location based on input measurement
    def UpdateLoc(self, measurement):
        self.x = measurement.x
        self.y = measurement.y
        self.z = measurement.z

        # Print current location, exploration status, and objective
        if self.isExploring is True:
            outstr = "Exploring: Drone %d moved to (%f|%f|%f), objective at (%f|%f|%f)" %(self.id, self.x, self.y,  self.z, self.xObjective, self.yObjective, self.zObjective)
        else:
            outstr =  "Not exploring: Drone %d moved to (%f|%f|%f), objective at (%f|%f|%f)" %(self.id, self.x, self.y,  self.z, self.xObjective, self.yObjective, self.zObjective)
        rospy.loginfo(outstr)

    # Calculates distance to objective (in feet based on GPS coordinates)
    def DistanceToObjective(self):
        xDist = (self.x - self.xObjective)/90*10000/0.0003048
        yDist = (self.y - self.yObjective)/90*10000/0.0003048
        zDist = (self.z - self.zObjective)/90*10000/0.0003048
        return ((xDist ** 2) + (yDist ** 2) + (zDist ** 2)) ** (1./2)

    # Updates drone status:
    # Collect new data, update self/maps
    # If drone is exploring
    #   If objective has been reached objective (proximity requirement is met)
    #       If any locations of interest have been detected so far
    #           Set drone to not exploring, set objective to previous location of interest with largest "interest" value
    #       Else if no locations of interest have been detected so far
    #           Drone is still exploring, set objective to next least explored location for current drone (based on ID)
    # Else if drone is not exploring
    #   If objective has been reached objective (proximity requirement is met)
    #       Set drone to exploring, set objective to next least explored location for current drone (based on ID)
    def UpdateStatus(self):
        currMeasurement = self.dataCollection.GetNewMeasurement()
        self.UpdateLoc(currMeasurement)
        self.maps.ProcessNewMeasurement(currMeasurement)
        if self.isExploring == True:
            if self.DistanceToObjective() < self.proximityReq:
                currValMap = self.maps.GetCurrentValueMapDist(self.t);
                if numpy.any(currValMap > 0):
                    (zLocs,xLocs,yLocs) = numpy.nonzero(numpy.amax(currValMap) == currValMap)
                    self.xObjective = xLocs[0]*self.xResolutionSize+self.bounds['xMin']
                    self.yObjective = yLocs[0]*self.yResolutionSize+self.bounds['yMin']
                    self.zObjective = zLocs[0]*self.zResolutionSize+self.bounds['zMin']
                    self.isExploring = False;
                    outstr =  "Drone %d reached target: not exploring, moving to previously found event, new objective at (%f|%f|%f)" %(self.id, self.xObjective, self.yObjective, self.zObjective)
                    rospy.loginfo(outstr)
                    self.dataCollection.UpdateObjective(self.xObjective, self.yObjective, self.zObjective)
                else:
                    (leastExploredX, leastExploredY, leastExploredZ) = self.maps.GetCurrentLeastExplored(self.t)
                    self.xObjective = leastExploredX*self.xResolutionSize+self.bounds['xMin']
                    self.yObjective = leastExploredY*self.yResolutionSize+self.bounds['yMin']
                    self.zObjective = leastExploredZ*self.zResolutionSize+self.bounds['zMin']
                    outstr = "Drone %d reached target: still exploring, moving to least explored, new objective at (%f|%f|%f), no event found yet" %(self.id, self.xObjective, self.yObjective, self.zObjective)
                    rospy.loginfo(outstr)
                    self.dataCollection.UpdateObjective(self.xObjective, self.yObjective, self.zObjective)
                time.sleep(5)
        else:
            if self.DistanceToObjective() < self.proximityReq:
                (leastExploredX, leastExploredY, leastExploredZ) = self.maps.GetCurrentLeastExplored(self.t)
                self.xObjective = leastExploredX*self.xResolutionSize+self.bounds['xMin']
                self.yObjective = leastExploredY*self.yResolutionSize+self.bounds['yMin']
                self.zObjective = leastExploredZ*self.zResolutionSize+self.bounds['zMin']
                self.isExploring = True
                outstr = "Drone %d reached target: now exploring, moving to least explored, new objective at (%f|%f|%f), no event found yet" %(self.id, self.xObjective, self.yObjective, self.zObjective)
                rospy.loginfo(outstr)
                self.dataCollection.UpdateObjective(self.xObjective, self.yObjective, self.zObjective)
                time.sleep(5)

class TestWaypointSubscriber:
    def __init__(self):
        self.sub = None
    def setSubscriber(self):
        if self.sub == None:
            self.sub = rospy.Subscriber("droneObj", GPS_Coord, self.WaypointReceive)
    def WaypointReceive(self,msg):
        outstr = "Fake receiver received drone published waypoint: new objective at (%f|%f|%f)" %(msg.longitude, msg.latitude, msg.height)
        rospy.loginfo(outstr)

class TestSensorPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('GasSensorData', GasSensorData, queue_size=1)
    def publishGasData(self, val):
        msg = GasSensorData()
        msg.value = val
        outstr = "Fake sensor sending fake data: %f" %(msg.value)
        rospy.loginfo(outstr)
        self.pub.publish(msg)

class TestGPSPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('droneGPS', GPS_Coord, queue_size=1)
    def publishGPS(self, longi, lat, height):
        msg = GPS_Coord()
        msg.longitude = longi
        msg.latitude = lat
        msg.height = height
        outstr = "Fake GPS sending fake location at (%f|%f|%f)" %(msg.longitude, msg.latitude, msg.height)
        rospy.loginfo(outstr)
        self.pub.publish(msg)

class FakeDataModule:
    def __init__(self, x, y, z, xObjective, yObjective, zObjective, bounds, resolutions):
        self.xObjective = xObjective
        self.yObjective = yObjective
        self.zObjective = zObjective
        self.x = x
        self.y = y
        self.z = z

        # Create random fake events based on exploration grid provided and number of fake events specified
        self.fakeEventNum = 5
        self.xNum = math.ceil( ( bounds['xMax'] - bounds['xMin'] ) / resolutions['x'] );
        self.yNum = math.ceil( ( bounds['yMax'] - bounds['yMin'] ) / resolutions['y'] );
        self.zNum = math.ceil( ( bounds['zMax'] - bounds['zMin'] ) / resolutions['z'] );
        self.fakeEventLocationsX = []
        self.fakeEventLocationsY = []
        self.fakeEventLocationsZ = []
        for i in range(self.fakeEventNum):
            self.fakeEventLocationsX.append((random.random()*(bounds['xMax'] - bounds['xMin']))+bounds['xMin'])
            self.fakeEventLocationsY.append((random.random()*(bounds['yMax'] - bounds['yMin']))+bounds['yMin'])
            self.fakeEventLocationsZ.append((random.random()*(bounds['zMax'] - bounds['zMin']))+bounds['zMin'])
        self.sensePub = TestSensorPublisher()
        self.GPSPub = TestGPSPublisher()
        self.waypointSub = TestWaypointSubscriber()

    # Fake movement for ROS testing
    # Returns GPS location of drone 1' towards objective location in x/y/z dimensions
    def FakeMove(self):
        if self.xObjective > self.x:
            self.x = self.x + 0.0003048/10000*90
        elif self.xObjective < self.x:
            self.x = self.x - 0.0003048/10000*90
        if self.yObjective > self.y:
            self.y = self.y + 0.0003048/10000*90
        elif self.yObjective < self.y:
            self.y = self.y - 0.0003048/10000*90
        if self.zObjective > self.z:
            self.z = self.z + 0.0003048/10000*90
        elif self.zObjective < self.z:
            self.z = self.z - 0.0003048/10000*90
        outstr = "Fake move: location is now (%f|%f|%f)" %(self.y, self.x,self.z)
        rospy.loginfo(outstr)
        self.GPSPub.publishGPS(self.y,self.x,self.z)

    # Fake sensor data value for standalone python testing
    # Returns a value based on proximity to fake locations of interest (sensing distance currently set to 20')
    def FakeVal(self):
        val = 0
        for i in range(self.fakeEventNum):
            xDist = (self.x - self.fakeEventLocationsX[i])/90*10000/0.0003048
            yDist = (self.y - self.fakeEventLocationsY[i])/90*10000/0.0003048
            zDist = (self.z - self.fakeEventLocationsZ[i])/90*10000/0.0003048
            temp = ((xDist ** 2) + (yDist ** 2) + (zDist ** 2)) ** (1./2)
            if temp < 20:
                val = val + (40/temp)
        if val > 0:
            outstr = "Fake event within range, val is %f, printing list of fake event locations:" %(val)
            rospy.loginfo(outstr)
            self.PrintFakeEventLocations()
        self.sensePub.publishGasData(val)
        time.sleep(3)

    # Prints out fake locations of events
    def PrintFakeEventLocations(self):
        for i in range(self.fakeEventNum):
            outstr = "[%f][%f][%f]," %(self.fakeEventLocationsX[i], self.fakeEventLocationsY[i], self.fakeEventLocationsZ[i]) 

class DroneGPSSubscriber:
    def __init__(self,x,y,z):
        self.longitude = y
        self.latitude = x
        self.height = z
        self.sub = None
    def setSubscribe(self):
        if self.sub == None:
            self.sub = rospy.Subscriber("droneGPS", GPS_Coord, self.GPSReceive)
    def GPSReceive(self,msg):
        self.longitude = msg.longitude
        self.latitude = msg.latitude
        self.height = msg.height
        outstr = "Drone received GPS data: location is now (%f|%f|%f)" %(msg.longitude, msg.latitude, msg.height)
        rospy.loginfo(outstr)
    def getGPS(self):
        outstr = "Get location issued: location is now (%f|%f|%f)" %(self.longitude, self.latitude, self.height)
        rospy.loginfo(outstr)
        return {'longitude':self.longitude,'latitude':self.latitude,'height':self.height}

class DroneWaypointPublisher:
    def __init__(self):
        self.longitude = 1000
        self.latitude = 1000
        self.height = 1000
        self.pub = rospy.Publisher('droneObj', GPS_Coord, queue_size=1)
    def setWaypoint(self, waypoint):
        self.longitude = waypoint['longitude']
        self.latitude = waypoint['latitude']
        self.height = waypoint['height']
        outstr = "Set waypoint issued: objective is now (%f|%f|%f)" %(self.longitude, self.latitude, self.height)
        rospy.loginfo(outstr)
    def publishWaypoint(self):
        msg = GPS_Coord()
        msg.longitude = self.longitude
        msg.latitude = self.latitude
        msg.height = self.height
        outstr = "Drone is publishing waypoint: objective is at (%f|%f|%f)" %(self.longitude, self.latitude, self.height)
        rospy.loginfo(outstr)
        self.pub.publish(msg)

class DroneSensorSubscriber:
    def __init__(self):
        self.value = 0
        self.sub = None
    def setSubscribe(self):
        if self.sub == None:
            self.sub = rospy.Subscriber("GasSensorData", GasSensorData, self.SensorReceive)
    def SensorReceive(self,msg):
        self.value = msg.value
        outstr = "Drone received sensor data: value is %f" %(self.value)
        rospy.loginfo(outstr)
    def getValue(self):
        outstr = "Get sensor value issued: last received value is %f" %(self.value)
        rospy.loginfo(outstr)
        return self.value

# Drone data aggregation class. Collects sensor data of interest for drone.
# NEEDS UPDATING. CONFIGURED FOR PYTHON STANDALONE TEST RUNS
class DroneDataAggregator:
    # Constructor:
    # NEEDS UPDATING. CONFIGURED FOR PYTHON STANDALONE TEST RUNS
    # Standalone python test implementation keeps track of drone location/objective, time, maintains
    # a map of fake events which determine fake sensor data output
    def __init__(self, x, y, z, xObjective, yObjective, zObjective, bounds, resolutions):
        self.t = 0
        self.xObjective = xObjective
        self.yObjective = yObjective
        self.zObjective = zObjective
        self.x = x
        self.y = y
        self.z = z

        rospy.init_node('drone_data_aggregator', anonymous = True)
        self.GPSReceiver = DroneGPSSubscriber(x,y,z)
        self.WaypointPublisher = DroneWaypointPublisher()
        self.WaypointPublisher.setWaypoint({'latitude':self.xObjective,'longitude':self.yObjective,'height':self.zObjective})
        self.WaypointPublisher.publishWaypoint()
        self.SensorReceiver = DroneSensorSubscriber()
        self.fakeData = FakeDataModule(x,y,z,xObjective, yObjective, zObjective, bounds, resolutions)

    # Should pull GPS/sensor data from NAVIO and return data in DroneMeasurement form
    # NEEDS UPDATING. Current standalone python test implementation performs a fake move and calculates fake values
    def GetNewMeasurement(self):
        self.GPSReceiver.setSubscribe()
        self.SensorReceiver.setSubscribe()
        self.fakeData.FakeMove()
        self.fakeData.FakeVal()
        GPSLoc = self.GPSReceiver.getGPS()
        SensorVal = self.SensorReceiver.getValue()
        # GET UPDATED TIME VALUE
        self.t = self.t + 1
        self.x = GPSLoc['latitude']
        self.y = GPSLoc['longitude']
        self.z = GPSLoc['height']
        return DroneMeasurement(self.x, self.y, self.z, self.t, SensorVal)

    # Update maintained drone current location
    def UpdateObjective(self, x, y, z):
        self.xObjective = x
        self.yObjective = y
        self.zObjective = z
        self.fakeData.waypointSub().setSubscribe()
        self.WaypointPublisher.setWaypoint({'latitude':self.xObjective,'longitude':self.yObjective,'height':self.zObjective})
        self.WaypointPublisher.publishWaypoint()

# Drone Mapping Class. Maintains the drone's ID, sniffradius, aging exponent, exploration area dimensions,
#   measurements (measured values for each grid location visited)
#   valueMapDist (locations of detected events of interest based on closest proximity)
#   sensedTimesDist (earliest time at which location was within sniff radius based on closest proximity)
#   minimumDistances (each grid location's minimum distance to drone detected during flight)
#   valueMapRecent (locations of detected events of interest based on most recent update within sniff radius)
#   sensedTimesRecent (earliest time at which location was within sniff radius based on most recent update within sniff radius)
#   x/y/z (distance grids used to determine current locations distance to all points in the grid)
class DroneObjectiveMapping:
    # Constructor. Requires drone ID and grid dimensions as input. The
    # maintained grids are initially set to empty.
    def __init__(self, bounds, resolutions, id):
        self.bounds = bounds;
        self.resolutions = resolutions
        self.id = id
        self.agingExponent = 0.01
        self.sniffRadius = 2
        self.measurements = []

        self.valueMapDist = numpy.zeros(( resolutions['z'], resolutions['x'], resolutions['y'] ))
        self.sensedTimesDist = numpy.zeros(( resolutions['z'], resolutions['x'], resolutions['y'] ))
        self.minimumDistances = 100000 * numpy.ones(( resolutions['z'], resolutions['x'], resolutions['y'] ))
        self.valueMapRecent = numpy.zeros(( resolutions['z'], resolutions['x'], resolutions['y'] ))
        self.sensedTimesRecent = numpy.zeros(( resolutions['z'], resolutions['x'], resolutions['y'] ))

        tempX = numpy.arange(resolutions['x'])
        tempY = numpy.arange(resolutions['y'])
        tempZ = numpy.arange(resolutions['z'])
        self.x, self.y, self.z = numpy.meshgrid(tempX,tempY,tempZ)
        self.x = numpy.transpose(self.x)
        self.y = numpy.transpose(self.y)
        self.z = numpy.transpose(self.z)

    # Update drone maps based on current drone location/time:
    # Get drone data (GPS location, time, sensor value data)
    # Create a grid of distances to each grid location based on current location
    # Create a grid of "drone sight": anything outside of sniffRadius is inf distance away
    # Update all maps based on time or distance
    # If enough time has passed (based on agingExponent), reset values in minimumDistances grid to Inf
    def UpdateMaps(self, measurement):
        [x,y,z] = self.GetCoordinateIndices( measurement )
        outstr = "Drone is at coordinates (%d|%d|%d)" %(x,y,z)
        rospy.loginfo(outstr)
        t = measurement.t
        val = measurement.value
        actualDistances = numpy.power(((self.x - x) ** 2) + ((self.y - y) ** 2) + ((self.z - z) ** 2), 1./2.)
        distances = actualDistances
        distGTsniffRadius = (distances > self.sniffRadius).astype(int)
        distLTEQsniffRadius = (numpy.logical_not(distGTsniffRadius)).astype(int)
        distances = (distances * distLTEQsniffRadius) + (10000 * distGTsniffRadius)

        actualGTminimum = (actualDistances > self.minimumDistances).astype(int)
        actualLTEQminimum = (numpy.logical_not(actualGTminimum)).astype(int)
        self.valueMapDist = self.valueMapDist * actualGTminimum + actualLTEQminimum * val
        self.sensedTimesDist = self.sensedTimesDist * actualGTminimum + actualLTEQminimum * t
        self.minimumDistances = self.minimumDistances * actualGTminimum + actualLTEQminimum * actualDistances
        #print(self.valueMapDist)
        #print(self.sensedTimesDist)
        #print(self.minimumDistances)

        self.valueMapRecent = self.valueMapRecent * distGTsniffRadius + distLTEQsniffRadius * val
        self.sensedTimesRecent = self.sensedTimesRecent * distGTsniffRadius + distLTEQsniffRadius * t
        #print(self.valueMapDist)
        #print(self.sensedTimesDist)
        #print(self.minimumDistances)

        # If non-0 values have been recorded, reset values that are too old
        if numpy.size(self.valueMapDist[ numpy.where( self.valueMapDist > 1 ) ]) > 0:
            ind = (numpy.absolute(self.sensedTimesDist - t) * self.agingExponent) > 1
            self.minimumDistances = (self.minimumDistances * numpy.logical_not(ind)) + (ind * 10000)

    # Returns the current valueMapDist after its been aged based on the agingExponent
    def GetCurrentValueMapDist(self, t):
        map = self.valueMapDist * numpy.exp((self.agingExponent*-1) * (t - self.sensedTimesDist))
        return map

    # Returns the current valueMapRecent after its been aged based on the agingExponent
    def GetCurrentValueMapRecent(self, t):
        map = self.valueMapRecent * numpy.exp((self.agingExponent*-1) * (t - self.sensedTimesRecent))
        return map

    # Returns the least explored locations based on minimumDistances. Sends different drones to different areas
    # based on minimumDistances grid
    def GetCurrentLeastExplored(self,t):
        distances = numpy.sort(self.minimumDistances, axis=None)[::-1]
        (leastExploredZ, leastExploredX, leastExploredY) = numpy.where((self.minimumDistances == distances[5*(self.id-1)]).astype(int) > 0)
        print "Least Explored value is %f, least explored loc is at (%f|%f|%f) with least explored value %f" %(distances[0], leastExploredX[0], leastExploredY[0], leastExploredZ[0], self.minimumDistances[leastExploredZ[0]][leastExploredX[0]][leastExploredY[0]])
        return (leastExploredX[0], leastExploredY[0], leastExploredZ[0])

    # Updates the measurements list. If the new measurement is close to a previous measurement, update the previous measurement.
    # Return the number of previous measurements that are close to the current measurement.
    def ReplaceSameLocation( self, measurement ):
        num = 0
        for (ind, data) in enumerate( self.measurements ):
            if self.measurements[ind].isClose( measurement ):
                self.measurements[ind].t = measurement.t
                self.measurements[ind].value = measurement.value
                num = num + 1
        return num

    # Update the maps based on the input measurement. If the new measurement is not seen before, append it to measurements. Otherwise
    # update the current measurements list only.
    def ProcessNewMeasurement( self, measurement ):
        self.UpdateMaps( measurement )
        if self.ReplaceSameLocation( measurement ) == 0:
            self.measurements.append(measurement)

    # Given an input measurement, return the respective grid coordinate
    def GetCoordinateIndices( self, measurement ):
        xIndex = math.ceil(( measurement.x - self.bounds['xMin'] ) / ( self.bounds['xMax'] - self.bounds['xMin'] ) * self.resolutions['x'])
        yIndex = math.ceil(( measurement.y - self.bounds['yMin'] ) / ( self.bounds['yMax'] - self.bounds['yMin'] ) * self.resolutions['y'])
        zIndex = math.ceil(( measurement.z - self.bounds['zMin'] ) / ( self.bounds['zMax'] - self.bounds['zMin'] ) * self.resolutions['z'])
        return [xIndex, yIndex, zIndex]
    
    # Sets current DroneObjectiveMapping object's valueMapDist, sensedTimesDist, valueMapRecent, sensedTimesRecent, and minimumDistances 
    # to that of DroneObjectiveMapping object "other"
    def Merge(self, other):
        self.valueMapDist = other.valueMapDist
        self.sensedTimesDist = other.sensedTimesDist
        self.minimumDistances = other.minimumDistances
        self.valueMapRecent = other.valueMapRecent
        self.sensedTimesRecent = other.sensedTimesRecent

    # Combines current DroneObjectiveMapping object with that of "other" into a new returned object.
    # For distance based grids, saves the values of the drone which has the lower minimumDistance value per grid location (closest visit)
    # For time based grids, saves the values of the drone which has the lower sesnedTimesRecent value per grid location (most recent visit)
    def Combine(self, other):
        rows,cols,height = self.minimumDistances.shape

        combined = DroneObjectiveMapping( self.bounds, self.resolutions, id )
        combined.eventMap = self.eventMap

        smallerDist = (self.minimumDistances < other.minimumDistances).astype(int)
        smallerDistInv = (numpy.logical_not(smallerDist)).astype(int)
        tempSelf = self.valueMapDist * smallerDist
        tempOther = other.valueMapDist * smallerDistInv
        combined.valueMapDist = tempSelf + tempOther
        tempSelf = self.sensedTimesDist * smallerDist
        tempOther = other.sensedTimesDist * smallerDistInv
        combined.sensedTimesDist = tempSelf + tempOther
        #numpy.savetxt("minDist1.txt",self.minimumDistances,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("minDist2.txt",other.minimumDistances,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("minDist1<minDist2.txt",smallerDist,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("minDist1>=minDist2.txt",smallerDistInv,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes1.txt",self.sensedTimesDist,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes2.txt",other.sensedTimesDist,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes1*(minDist1<minDist2).txt",tempSelf,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes2*(minDist1>=minDist2).txt",tempOther,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("combined_sensedTimes.txt",combined.sensedTimesDist,fmt='%1.3f',delimiter=",")
        tempSelf = self.minimumDistances * smallerDist
        tempOther = other.minimumDistances * smallerDistInv
        combined.minimumDistances = tempSelf + tempOther
        #numpy.savetxt("combined_minDist.txt",combined.minimumDistances,fmt='%1.3f',delimiter=",")

        moreRecentSensedTimes = (self.sensedTimesRecent >= other.sensedTimesRecent).astype(int)
        moreRecentSensedTimesInv = (numpy.logical_not(moreRecentSensedTimes)).astype(int)
        tempSelf = self.valueMapRecent * moreRecentSensedTimes
        tempOther = other.valueMapRecent * moreRecentSensedTimesInv
        combined.valueMapRecent = tempSelf + tempOther
        tempSelf = self.sensedTimesRecent * moreRecentSensedTimes
        tempOther = other.sensedTimesRecent * moreRecentSensedTimesInv
        combined.sensedTimesDist = tempSelf + tempOther
        #numpy.savetxt("minDist1.txt",self.minimumDistances,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("minDist2.txt",other.minimumDistances,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("minDist1<minDist2.txt",smallerDist,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("minDist1>=minDist2.txt",smallerDistInv,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes1.txt",self.sensedTimesDist,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes2.txt",other.sensedTimesDist,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes1*(minDist1<minDist2).txt",tempSelf,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("sensedTimes2*(minDist1>=minDist2).txt",tempOther,fmt='%1.3f',delimiter=",")
        #numpy.savetxt("combined_sensedTimes.txt",combined.sensedTimesDist,fmt='%1.3f',delimiter=",")
    
        return combined

# Currently unused, may be used later for drone-to-drone communication
class DroneCommunicator:
    def __init__(self):
        self.ipAddress = '';
        self.port = 5005;
        self.bufferSize = 1024;

        try:
            self.socketHandle = socket.socket( socket.AF_INET, socket.SOCK_STREAM );
            self.socketHandle.setsockopt( socket.SOL_SOCKET, socket.SO_REUSEADDR, 1 )
        except socket.error as msg:
            s = None;

        self.StartServer();
        self.WaitForDroneConnection();

    def StartServer(self):
        try:
            self.socketHandle.bind( ( self.ipAddress, self.port ) );
            self.socketHandle.listen( 1 );
        except socket.error as msg:
            self.socketHandle.close();
            self.socketHandle = None;

        if self.socketHandle is None:
            print 'Server Socket Creation Failed';

    def WaitForDroneConnection(self):
        self.connectionHandle, self.droneAddress = self.socketHandle.accept();
        print 'Connected by ', self.connectionHandle

    def ReceiveNewData(self):
        data = self.connectionHandle.recv( self.bufferSize );
        return data;

    def SendNewData(self, data):
        self.connectionHandle.send( data );

# Holder class for measurements, maintains GPS location, time, and value variables
class DroneMeasurement:
    def __init__( self, x, y, z, tTime, value ):
        self.proximityReq = 1;
        self.x = x;
        self.y = y;
        self.z = z;
        self.t = tTime;
        self.value = value;

    # Returns whether input measurement is close to current measurement (within proximity requirement)
    def isClose(self,measurement):
        if ( ( self.x - measurement.x ) ** 2 + ( self.y - measurement.y ) ** 2 + ( self.z - measurement.z )** 2) ** (1./2) <= self.proximityReq:
            return True;
        else:
            return False;

    # Prints measurement to JSON format
    def toJSON(self):
        return json.dumps( self, default=lambda o: o.__dict__ );

# Main function for each drone to run
def main():
    # Desired grid dimensions in feet
    xDiam = 90
    yDiam = 90
    zDiam = 45

    # Desired grid resolution, number of drones, and communication radius (drone-to-drone range)
    resolutions = {'x':30,'y':30,'z':15}
    numDrones = 1
    #commRadius = 5

    # Start location in X/Y GPS coordinates, Z in feet)
    xStart = -117.213836
    yStart = 32.865174
    zStart = 130

    #~10000km per 90 degrees -> ~0.00013716 degrees per 50feet

    #Need to handle manually setting each drone's ID
    drone = Drone(xStart, yStart, zStart, xDiam, yDiam, zDiam, resolutions, 1)
    drone.maps.agingExponent = 0
    print "Start!"

    for t in range(10000):
        drone.UpdateStatus()
        time.sleep(0.5)

if __name__ == "__main__":
    main()
