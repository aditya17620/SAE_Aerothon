#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
# 192.168.141.18:14551
"""

######################################################################
#   IMPORT LIBRARIES
######################################################################

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import time
import math
import cv2
import utils
from tflite_support.task import core
from tflite_support.task import processor
from tflite_support.task import vision
import RPi.GPIO as GPIO
# import RPi.GPIO as GPIO
# import rospy

######################################################################
#   GLOBAL VAIRABLES
######################################################################

global cameraX
global cameraY
global centerX
global centerY
global hastargetbeendetected
global distance_from_waypoint
global listofwaypoints
global k
global v

# FOR MIT GROUND
listofwaypoints = ([13.344069, 74.793536], [13.343796, 74.793661], [13.343928, 74.793960], [13.344228, 74.793785])

# FOR  MIT FOOTBALL GROUND
#listofwaypoints = ([13.342471, 74.792426], [13.342142, 74.792377], [13.342194, 74.792715], [13.342541, 74.792707])

# FOR SIMULATION
#listofwaypoints = ([-35.36316, 149.16523], [-35.363244, 149.16527], [-35.361354, 149.165218], [-35.363244, 149.168801])

# FOR OUTISDE FORMULA
#listofwaypoints = ([13.347669, 74.792129], [13.347934, 74.792134], [13.347669, 74.792129])

# # FOR TMR GROUND NEAR SURESH
# listofwaypoints = ([])

############################## AEROTHON COMP #################################################################

# FIELD 1 POINTS 1 - 7
#listofwaypoints = [(13.39499, 77.7311609), (13.394727, 77.7311024), (13.394185, 77.7310135), (13.393622, 77.7309197), (13.39352, 77.7313704), (13.394631, 77.7315509), (13.394896, 77.731609)]

# FIELD 2 POINTS 8 - 14
#listofwaypoints = [(13.394887, 77.7316545), (13.394622, 77.7316004), (13.393506, 77.7314371), (13.393385, 77.7318823), (13.394057, 77.732051), (13.39454, 77.7320772), (13.394808, 77.7321099)]

#############################################################################################################
cameraX = 640/2
cameraY = 480/2
centerX = None
centerY = None
hastargetbeendetected = False
distance_from_waypoint = 1000
k = 0
v = 0

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("PRINTMSG: Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("PRINTMSG: Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" PRINTMSG: Waiting for arming...")
        time.sleep(1)

    print("PRINTMSG: Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" PRINTMSG: Altitude: ", vehicle.location.global_relative_frame.alt)
        # Trigger just below target alt.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print("PRINTMSG: Reached target altitude")
            break
        time.sleep(1)


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def condition_yaw(heading, clockdir, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.
    By default the yaw of the vehicle will follow the direction of travel. After setting 
    the yaw using this function there is no way to return to the default yaw "follow direction 
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)
    For more information see: 
    http://copter.ardupilot.com/wiki/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_condition_yaw
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        clockdir,    # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
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
    vehicle.send_mavlink(msg)


def navigate_to_waypoint_and_check(waypointcoord):
    """
    Navigates vehicle from current location to given waypoint while maintaining altitude
    location of waypont is given as metres of offset from present vehicle location
    """
    global distance_from_waypoint

    dlat = waypointcoord[0]
    dlon = waypointcoord[1]
    dalt = vehicle.location.global_frame.alt

    waypoint = LocationGlobal(dlat, dlon, dalt)
    

    j = 0
    while(vehicle.mode.name == "GUIDED") :
        ret, frame = cap.read()
        target_detection(frame)
        if hastargetbeendetected == True : 
            # align_and_drop()
            break
        j = j + 1
        #cv2.imwrite('/home/aerothon/examples/')                                               ################### ADD FILE PATH
        distance_from_waypoint = get_distance_metres(vehicle.location.global_frame, waypoint)
        # if (j%100 == 0) : print('PRINTMSG: distance from waypoint: ', distance_from_waypoint)
        vehicle.simple_goto(waypoint, groundspeed=1)
        if distance_from_waypoint <= 0.95 :
            print('reached target')
            break


def target_detection(frame) :

    """
    Continuously run inference on images acquired from the camera.

    Args:
    model: Name of the TFLite object detection model.
    camera_id: The camera id to be passed to OpenCV.
    width: The width of the frame captured from the camera.
    height: The height of the frame captured from the camera.
    num_threads: The number of CPU threads to run the model.
    enable_edgetpu: True/False whether the model is a EdgeTPU model.
    """

    global centerX
    global centerY
    global hastargetbeendetected
    global k
    global v

    v = v + 1
    
    if (frame is not None) :

        image = cv2.flip(frame, 1)

        # Convert the image from BGR to RGB as required by the TFLite model.
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Create a TensorImage object from the RGB image.
        input_tensor = vision.TensorImage.create_from_array(rgb_image)

        # Run object detection estimation using the model.
        detection_result = detector.detect(input_tensor)

        image = utils.visualize(image, detection_result)
        cv2.imwrite('~/examples/lite/examples/object_detection/raspberry_pi/ft3/Frames' + str(v) + '.jpg', image)

        if len(detection_result.detections) > 0 :
            hastargetbeendetected = True
            centerX = detection_result.detections[0].bounding_box.origin_x
            centerY = detection_result.detections[0].bounding_box.origin_y
            return 
        
        else :
            hastargetbeendetected = False
            centerX = None
            centerY = None
            return 

    else : 
        print('PRINTMSG: ERROR: frames not found, camera not released')
        # if (k%50 == 0) : print('PRINTMSG: frame is NONE')
        return 


def align_and_drop() :
    """
    aligns drone with center of detected target
    """

    global centerX
    global centerY
    global cameraX
    global cameraY

    print('PRINTMSG: beginning alignment')
    offsetX = (centerX - cameraX)**2
    offsetY = (centerY - cameraY)**2

    if (offsetY >= 144) or (offsetX >= 144) :
        if (centerX is not None) and (centerY is not None) :
            print('PRINTMSG: target offset: ', (offsetX, offsetY))
            while (offsetX >= 144) or (offsetY >= 144) :
                ret, frame = cap.read()
                target_detection(frame)
                offsetX = (centerX - cameraX)**2
                offsetY = (centerY - cameraY)**2            

                lenX = (centerX - cameraX)
                lenY = (centerY - cameraY)

                yaw_deg = math.atan2(lenY, lenX) * (180 / math.pi)
                if (yaw_deg < 0) : yaw_deg  = 360 + yaw_deg
                yaw_drone = 90 + yaw_deg
                print('PRINTMESG: yaw degree: ', yaw_drone)
                time.sleep(5)

                if yaw_drone > 180 :
                    yaw_drone = 360 - yaw_drone
                    condition_yaw(yaw_drone, -1, relative = True)
                    time.sleep(3)

                elif yaw_drone > 10 :
                    print('PRINTMSG: yaw degree: ',yaw_drone)
                    condition_yaw(yaw_deg, 1, relative=True)
                    time.sleep(3)
                send_ned_velocity(1,0,0)
            
        else : print('PRINTMSG: TARGET NOT DETECTED')


    print('PRINTMSG: dropping target')
    

def servo_drop() :
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(17, GPIO.OUT)
    pwm=GPIO.PWM(17, 50)
    pwm.start(0)
    time = 0.01

    duty=10.0

    pwm.ChangeDutyCycle(duty)
    time.sleep(3)

    while duty >4.5:
        pwm.ChangeDutyCycle(duty)
        time.sleep(time)
        duty = duty - 0.1

    GPIO.cleanup()


########################################################################################
#         MAIN FUNCTION
########################################################################################

vehicle = connect(ip='/dev/serial/by-id/usb-ArduPilot_fmuv3_280052000451353431383238-if00', wait_ready=True, baud=921600)
#vehicle = connect(ip='127.0.0.1:14550', wait_ready=True, baud=921600)
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
cap = cv2.VideoCapture(0)

# Initialize the object detection model
base_options = core.BaseOptions(file_name='second.tflite', use_coral=False, num_threads=4)
detection_options = processor.DetectionOptions(max_results=3, score_threshold=0.3)
options = vision.ObjectDetectorOptions(base_options=base_options, detection_options=detection_options)
detector = vision.ObjectDetector.create_from_options(options)

original_location = vehicle.location.global_frame
print('PRINTMSG: original location initialised: ', original_location)

arm_and_takeoff(30)

for waypointcoord in listofwaypoints :
    print('PRINTMSG: next waypoint: ', waypointcoord)
    navigate_to_waypoint_and_check(waypointcoord)

    if hastargetbeendetected == True : 
        k += 1
        print('PRINTMSG: K value: ', k)
        target_located = vehicle.location.global_frame
        print('PRINTMSG: target detected at vehicle location: ', (target_located.lat, target_located.lon))
        
        ## halt at present location
        vehicle.simple_goto(target_located)
        distance_from_waypoint = get_distance_metres(vehicle.location.global_frame, target_located)
        while distance_from_waypoint >= 0.9 :
            distance_from_waypoint = get_distance_metres(vehicle.location.global_frame, target_located)
            time.sleep(1)
            if distance_from_waypoint <= 0.9 : break
        print('PRINTMSG: vehicle centered at location', (target_located.lat, target_located.lon))
        print('PRINTMSG: target has been detected at: ', (centerX, centerY))

        ## align with target
        align_and_drop()

        ## decrease altitude by 10
        templocation = vehicle.location.global_frame
        templocation.alt = templocation.alt - 10
        vehicle.simple_goto(templocation)
        distance_from_waypoint = get_distance_metres(vehicle.location.global_frame, templocation)
        while distance_from_waypoint >= 0.9:
            distance_from_waypoint = get_distance_metres(vehicle.location.global_frame, templocation)
            time.sleep(1)
            print('PRINTMSG: descending to 20m ', distance_from_waypoint)
            if distance_from_waypoint <= 0.9 : break
        print('PRINTMSG: descended to 20m alt')

        ## align with target again
        align_and_drop()
        
        ## release payload
        servo_drop()

        ## increase altitude by 10
        templocation = vehicle.location.global_frame
        templocation.alt = templocation.alt + 10
        vehicle.simple_goto(templocation)
        distance_from_waypoint = get_distance_metres(vehicle.location.global_frame, templocation)
        while distance_from_waypoint >= 0.9:
            distance_from_waypoint = get_distance_metres(vehicle.location.global_frame, templocation)
            time.sleep(1)
            print('PRINTMSG: climbing to 30m ', distance_from_waypoint)
            if distance_from_waypoint <= 0.9 : break

        print('PRINTMSG: climbed 30m: proceeding to RTL')
        break

## RTL
print("PRINTMSG: Returning to Launch")
vehicle.mode = VehicleMode("RTL")
print('PRINTMSG: VEHICLE MODE: ', vehicle.mode) 

while vehicle.armed :
    vehicle.mode = VehicleMode("RTL")

cap.release()
cv2.destroyAllWindows()
print('PRINTMSG: camera released')
print('PRINTMSG: VEHICLE DISARMED')
