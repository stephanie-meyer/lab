#!/usr/bin/env python
#Modified by Stephanie Meyer 2-16-18
#This reduces the visible range of given lidars to 180 degrees (or any other given arc)
#This code will work for any laserscan message where the center of the desired filtered arc is at the center of the original scan
#This code has been verified for the Velodyne (originally 360*) and for the hokuyo urg (origianlly 180*), where the 
	#"front" direction for the lidar is the center of the filtered arc

import roslib
#import roslib.load_manifest('gavlab_atrv_2dnav') #requires the gavlab_atrv_2dnav set

import rospy
from sensor_msgs.msg import LaserScan
import math 
import sys

#Let's try setting up filteredArcDeg as a global variable here
filteredArcDeg = 180 #constant, 180deg default

def callbackSmart(msg, pub):
    #TODO: consider even/odd centers, and finding the correct number of readings to drop. as-is, it should approximate 180* (centered about the center of a scan)
    #assuming each scan starts at the far edge of the traversed arc, passes parallel to the "front" direction in the dead center of the scan arc,
        #and then ends at the opposite far edge of the traversed arc, then:
        #
        # # readings = (maxAngle-minAngle)/(angleInc)
        #
        # (# readings)/2 = center or front
        # center - (90 deg)/angleInc = one arc end (180 deg arc)
        # center + (90 deg)/angleInc = the other arc end (180 deg arc)
        # the absolue value of the first end should be the number of readings to drop from either end of the original arc
    
    #rospy.loginfo("Filter to: %s degrees", filteredArcDeg)
    
    #filteredArcDeg = 180 #constant - in degrees #should be 180

    minAngle = msg.angle_min
    maxAngle = msg.angle_max
    angleInc = msg.angle_increment

    originalArc = (maxAngle - minAngle)
    filteredArc = math.radians(filteredArcDeg % 360) #this makes sure the arc given is mapped to a 360 deg circle, and converts to radians

    if(filteredArc < originalArc): #otherwise, the original arc is already smaller than or the same size of filtered arc
    	center = originalArc/2  #TODO:think about what to do about finding actual front in cases with odd/even num readings 
    	arcToDrop = abs(center - filteredArc/2) #TODO:Again, handle even/odd cases appropriatly

    	numReadingsToDrop = int(arcToDrop/angleInc) #this rounds down, so filtered arc might be slightly more than 180*
    	
        ranges = list(msg.ranges)

    	for i in range(numReadingsToDrop): #range needs an int as a param
     	   ranges[i] = 0.0 # set the first set of points to truncate to 0 (start at i = 0, and end when the end of the arc is found)
    	offset = len(ranges) - numReadingsToDrop #use this to set the last (num points to drop) points to 0
    	for i in range(numReadingsToDrop):
     	   ranges[offset+i] = 0.0 
   	msg.ranges = tuple(ranges)
    pub.publish(msg)

#This callback function will only work for the velodyne and hokuyo lidars at present
    #and will produce a 180* arc centered about the center of the original arc (for these lidars, centered about lidar front)

    #hokuyo (from launch file urg_lidar.launch): minAngle: -1.5707963; maxAngle: 1.5707963 (if radians, in degrees: -90, 90)
    	#increment unknown... front = 0
    #velodyne: (from .cfg in velodyne_laserscan, and VelodyneLaserScan.cpp and VLP16_points.launch):
    	# angle_increment = 0.007 rad = 0.40107 deg (.cfg)
    	# angleMin = -pi rad = -180 deg
    	# angleMax = pi rad = 180 deg
def callbackHardcoded(msg, pub):  
    numDropV = 225 #constant - (pi/2)/0.07 - rounded up to make range slightly less than 180*
    numDropH = 0 #constant - the hokuyo is already at 180* centerered about front, so should be 0

    h = msg.header
    frame = h.frame_id

    if(frame == "velodyne_link"):
        isVelodyne = True
        isHokuyo = False
    elif(frame == "hokuyo_link"):
        isVelodyne = False
        isHokuyo = True
    else:
        isVelodyne = False
        isHokuyo = False

    if(isVelodyne):
        numDrop = numDropV
    elif(isHokuyo):
        numDrop = numDropH
    #else:
        #some sort of error - device is neither velodyne nor hukoyo, or the frame id's aren't named correctly

    ranges = list(msg.ranges)
    for i in range(numDrop):
        ranges[i] = 0.0 # set the first set of points to truncate to 0 (start at i = 0, and end when the end of the arc is found)
    offset = len(ranges) - numDrop #use this to set the last (num points to drop) points to 0
    for i in range(numDrop):
        ranges[offset+i] = 0.0 
    msg.ranges = tuple(ranges)
    pub.publish(msg)

def main():
    global filteredArcDeg
    rospy.init_node('laser_filter')

    #try defining filtered arc deg as global var from command line args here
    numArgs = len(sys.argv)
    if(numArgs == 2):
        if(str.isdigit(sys.argv[1])):
            filteredArcDeg = int(sys.argv[1])
            rospy.loginfo("Filter to: %s degrees", sys.argv[1])
    ##THIS DIDN'T WORK -- GLOBAL VAR WAS NOT CHANGED WITH THE ABOVE LOCAL CODE
    
    pub1 = rospy.Publisher("scan_filtered", LaserScan) #adding a queue_size here is recommended, but not required. 
     #queue_size allows for asynchronous behavior; ie queueing messages in a buffer to ensure no "packages are dropped"
    
    rospy.Subscriber("scan", LaserScan, callbackSmart, pub1) #change this to dual_scan (or whatever that topic v and h publish to is)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass