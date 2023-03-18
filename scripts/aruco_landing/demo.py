#!/usr/bin/env python3

import pymavlink.mavutil as mavutil

import asyncio
from mavsdk import System
from mavsdk.offboard import (
    OffboardError, PositionNedYaw, VelocityBodyYawspeed)

import landing

import cv2

mavlink_addr = "127.0.0.1"
mavlink_port = 14750
src_sys = 201

mavsdk_port = 14540
front_offset = 0.3
side_offset = -0.2
rotation_offset = 70
altitude = 7

descent_speed = 0.6
aruco_visualization = True

async def run():
    #  PyMavlink connection for cover openning and closing.
    mav = mavutil.mavlink_connection(
        'udpout:' + mavlink_addr + ":" + str(mavlink_port), source_system=src_sys)
    # MAVSDK connection for drone control.
    drone = System()

    lander = landing.Landing(drone, descent_speed)

    await drone.connect(system_address="udp://:" + str(mavsdk_port))

    print("-- Arming")
    await drone.action.arm()
    print("-- Taking off")
    await drone.action.takeoff()

    await asyncio.sleep(10)
    print("-- Starting offboard")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(
            f"Starting offboard mode failed with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return
    print("-- Move to position--")
    await drone.offboard.set_position_ned(PositionNedYaw(front_offset, side_offset, -altitude, rotation_offset))
    await asyncio.sleep(5)
    #print("-- Open DronePort cover --")
    #mav.mav.param_set_send(src_sys, 1, b'cover', 2, 0)
    #await asyncio.sleep(5)
    print("-- Open Video Capture --")
    cap = cv2.VideoCapture("udpsrc port=5601 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert ! appsink emit-signals=true sync=false max-buffers=16 drop=false", cv2.CAP_GSTREAMER)

    #cap = cv2.VideoCapture("udpsrc port=5601 ! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert ! appsink emit-signals=true sync=false max-buffers=8 drop=true", cv2.CAP_GSTREAMER)
    #cap = cv2.VideoCapture("udpsrc port=5601 ! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=1000 appsink emit-signals=true sync=false", cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print('VideoCapture not opened')
        exit(-1)

    # for OpenCV version 4.7.0 
    #arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    #arucoParams = cv2.aruco.DetectorParameters()

    while True:
        ret, frame = cap.read()


        if not ret:
            print('frame empty')
            break

        corners, center, id = lander.process_frame(frame)
        if corners is None or len(corners) == 0:
            continue

        (topLeft, topRight, bottomRight, bottomLeft) = corners
        # convert each of the (x, y)-coordinate pairs to integers
        #topRight = (int(topRight[0]), int(topRight[1]))
        #bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        if ((bottomLeft[1] - topLeft[1]) > 0.75 * frame.shape[0]):
            await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0, 0))
            cap.release()
            cv2.destroyAllWindows()
            break

        moveX, moveY, rotation = lander.compute_move(
            frame.shape, corners, center)
        #print(moveX, moveY,rotation)
        await lander.apply_move(moveX, moveY, rotation)

        if (aruco_visualization):
            frame = lander.draw_aruco(frame, corners)
            cv2.imshow('aruco_visualization', frame)
            if cv2.waitKey(1) & 0XFF == ord('q'):
                break

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(
            f"Stopping offboard mode failed with error code: {error._result.result}")
    await asyncio.sleep(2)
    print ("-- Landing --")
    await drone.action.land()
    await asyncio.sleep(10)
    #await drone.action.disarm()
    #await asyncio.sleep(10)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
