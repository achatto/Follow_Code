# Import the required modules
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math

#Set up option parsing to get connection string
import argparse 

import dlib
import cv2
import get_points
import numpy


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

        
    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("GUIDED")
    vehicle.armed   = True    

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:      
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print " Altitude: ", vehicle.location.global_relative_frame.alt 
        #Break and return from function just below target altitude.        
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            print "Reached target altitude"
            break
        time.sleep(1)
        
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
   
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def run(source=0, dispLoc=False):
    # Create the VideoCapture object
    cam = cv2.VideoCapture()
    cam.open(source)
    # If Camera Device is not opened, exit the program
    if not cam.isOpened():
        print "Video device or file couldn't be opened"
        exit()
    


    print "Press key `p` to pause the video to start tracking"
    while True:
        # Retrieve an image and Display it.
        retval, img = cam.read()
        if not retval:
            print "Cannot capture frame device"
            exit()
        if(cv2.waitKey(10)==ord('p')):
            break
        #cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Image")
        cv2.imshow("Image", img)
    cv2.destroyWindow("Image")

    # Co-ordinates of objects to be tracked 
    # will be stored in a list named `points`
    points = get_points.run(img) 

    deadzone_height = numpy.size(img, 0)/2
    deadzone_width = numpy.size(img, 1)/2

    print ('width =  ',deadzone_height ,'height =  ',deadzone_width)

    if not points:
        print "ERROR: No object to be tracked."
        exit()
    
    cv2.namedWindow("Image")
    cv2.imshow("Image", img)

    # Initial co-ordinates of the object to be tracked 
    # Create the tracker object
    tracker = dlib.correlation_tracker()
    # Provide the tracker the initial position of the object
    tracker.start_track(img, dlib.rectangle(*points[0]))

    while True:
        # Read frame from device or file
        retval, img = cam.read()
        if not retval:
            print "Cannot capture frame device | CODE TERMINATING :("
            exit()
        # Update the tracker  
        tracker.update(img)
        # Get the position of the object, draw a 
        # bounding box around it and display it.
        rect = tracker.get_position()
        pt1 = (int(rect.left()), int(rect.top()))
        pt2 = (int(rect.right()), int(rect.bottom()))
        center = ((pt1[0]+pt2[0])/2 , (pt1[1]+pt2[1])/2)
        print (center)

        if (deadzone_width - center[0]) > deadzone_width/2:
        	print('move left')
        	#send_ned_velocity(-1,0,0,5)
        	#time.sleep(5)

        if (deadzone_width - center[0]) < -deadzone_width/2:
        	print('move right')
        	#send_ned_velocity(1,0,0,5)
        	#time.sleep(5)

        if (deadzone_height - center[1]) > deadzone_height/2:
        	print('move forward')
        	#send_ned_velocity(0,1,0,5)
        	#time.sleep(5)
        if (deadzone_height - center[1]) < -deadzone_height/2:
        	print('move back')
        	#send_ned_velocity(0,-1,0,5)
        	#time.sleep(5)


        cv2.rectangle(img, pt1, pt2, (255, 255, 255), 3)
        #print "Object tracked at [{}, {}] \r".format(pt1, pt2),
        if dispLoc:
            loc = (int(rect.left()), int(rect.top()-20))
            txt = "Object tracked at [{}, {}]".format(pt1, pt2)
            cv2.putText(img, txt, loc , cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
        cv2.namedWindow("Image")
        cv2.imshow("Image", img)
        
        # Continue until the user presses ESC key
        if cv2.waitKey(1) == 27:
        	print("Setting LAND mode...")
        	vehicle.mode = VehicleMode("LAND")
        	#Close vehicle object before exiting script
        	print "Close vehicle object"
        	vehicle.close()
        	print("Completed")
        	break

    # Relase the VideoCapture object
    cam.release()




if __name__ == "__main__":
    # Parse command line arguments
    #parser = argparse.ArgumentParser()
    #group = parser.add_mutually_exclusive_group(required=True)
    #parser.add_argument('-d', "--deviceID", help="Device ID")
    #group.add_argument('-v', "--videoFile", help="Path to Video File")
    #parser.add_argument('-l', "--dispLoc", dest="dispLoc", action="store_true")
    #args = parser.parse_args()
    #print args.deviceID
    # Get the source of video

    parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
    parser.add_argument('--connect', default='127.0.0.1:14550',
                   help="vehicle connection target. Default '127.0.0.1:14550'")
    args = parser.parse_args()

    # Connect to the Vehicle
    print 'Connecting to vehicle on: %s' % args.connect
    vehicle = connect(args.connect, wait_ready=True ,baud=57600)
    
    arm_and_takeoff(0)
    vehicle.groundspeed=5
    #source = int(0)
    run(0)