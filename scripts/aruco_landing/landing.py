#!/usr/bin/env python3
import cv2
from mavsdk.offboard import VelocityBodyYawspeed

class Landing:

    def __init__(self, drone, descent):
        self.drone = drone
        # Descent speed
        self.descent = descent 
        pass

    def process_frame(self, frame):
        # Assumption -- only one aruco is in the frame.
        #arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        #arucoParams = cv2.aruco.DetectorParameters_create()   

        # for OpenCV version 4.7.0 
        arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters()

        (detected_corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams) 
        #print(corners, ids, rejected)
        # verify *at least* one ArUco marker was detected
        centers = []
        corners = [] 

        if len(detected_corners) > 0:
        # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(detected_corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners.append(markerCorner.reshape((4, 2)))
                (topLeft, topRight, bottomRight, bottomLeft) = markerCorner.reshape((4, 2))
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1])) 

                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                centers.append((cX, cY))
            print(corners, centers, ids)
            return corners, centers, ids
        return None, None, None

    def compute_move(self, frame_shape, corners, center):
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        diffX = -((frame_shape[1]/2.0) - center[0])
        diffY = ((frame_shape[0]/2.0) - center[1])

        u_vec = (bottomLeft[0] - topLeft[0], bottomLeft[1] - topLeft[1])

        moveX = (1.2*diffY/((frame_shape[1]/2.0))) 
        moveY = (1.2*diffX/((frame_shape[1]/2.0)))
        rotation = -u_vec[0]/5.0

        return moveX, moveY, rotation

    async def apply_move(self, moveX, moveY, rotation):
        await self.drone.offboard.set_velocity_body(VelocityBodyYawspeed(moveX, moveY, self.descent, rotation));

    def draw_aruco(self, frame, corners):
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))

        cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
        cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
        cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
        cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
        # compute and draw the center (x, y)-coordinates of the ArUco
        # marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

        cv2.putText(frame, str(id),
                    (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 0), 2)

        cv2.circle(frame, topLeft, 5, (255, 0, 0), -1)

        return frame