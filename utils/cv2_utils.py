""" Copyright (c) Mohsen Azimi, 2022"""

# Importing Libraries
import cv2  # Import the OpenCV library


# import numpy as np  # Import Numpy library
# import time
# import os
# import argparse

# from utils import UGV

# robot = UGV()
# import time
class ArUco_marker:
    def __init__(self, desired_aruco_dictionary="DICT_4X4_1000"):
        self.desired_aruco_dictionary = desired_aruco_dictionary
        # The different ArUco dictionaries built into the OpenCV library.

        self.ARUCO_DICT = {
            "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
            "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
            "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
            "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
            "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
            "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
            "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
            "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
            "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
            "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
            "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
            "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
            "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
            "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
            "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
            "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
            "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
        }

        # if ARUCO_DICT.get(desired_aruco_dictionary, None) is None:
        #     print("[INFO] ArUCo tag of '{}' is not supported".format(
        #         args["type"]))
        #     sys.exit(0)

    def detect_obj(self, frame):
        # Load the ArUco dictionary
        # Detect ArUco markers in the video frame
        this_aruco_dictionary = cv2.aruco.Dictionary_get(self.ARUCO_DICT[self.desired_aruco_dictionary])
        this_aruco_parameters = cv2.aruco.DetectorParameters_create()
        # print("[INFO] detecting '{}' markers...".format(self.desired_aruco_dictionary))

        (corners, ids, rejected) = cv2.aruco.detectMarkers(
            frame, this_aruco_dictionary, parameters=this_aruco_parameters)

        return (corners, ids, rejected)


def im_show(h, w, *args, **kwargs):
    """This funciton helps to keep the main.py clean!"""
    for arg in args:
        cv2.namedWindow(arg[0], cv2.WINDOW_NORMAL)
        cv2.resizeWindow(arg[0], h, w)
        img = arg[1]
        if img.ndim==2:
            img = cv2.applyColorMap(cv2.convertScaleAbs(arg[1], alpha=.03), cv2.COLORMAP_JET) # colorize depth

        cv2.imshow(arg[0], img)

    for key, value in kwargs.items():
        if key == 'show_bbox':
            pass
            # print('show_bbox')




if __name__ == '__main__':
    marker = ArUco_marker()
    # print(marker)

"""
# ................................................
        # Start the video stream
        cap = cv2.VideoCapture(2)
        # arduino = serial.Serial(port='COM11', baudrate=9600, timeout=.1)
        moves = ["moveForward", "moveSidewaysLeft", "moveSidewaysRight", "moveSidewaysLeft", "moveRightForward",
                 "moveLeftForward", "moveRightBackward", "moveLeftBackward", "rotateRight", "rotateLeft", "rotateLeft",
                 "stopMoving"]

    while True:

        # Capture frame-by-frame
        # This method returns True/False as well
        # as the video frame.
        ret, frame = cap.read()

        # Detect ArUco markers in the video frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
            frame, this_aruco_dictionary, parameters=this_aruco_parameters)

        # Check that at least one ArUco marker was detected
        if len(corners) > 0:
            # Flatten the ArUco IDs list
            ids = ids.flatten()

            # Loop over the detected ArUco corners
            for (marker_corner, marker_id) in zip(corners, ids):
                # Extract the marker corners
                #
                robot.move(direction=moves[marker_id - 1], speed=30, motion_time=100).to_arduino()

                # print(moves[marker_id-1])
                corners = marker_corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners

                # Convert the (x,y) coordinate pairs to integers
                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                # Draw the bounding box of the ArUco detection
                cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)

                # Calculate and draw the center of the ArUco marker
                center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)

                # Draw the ArUco marker ID on the video frame
                # The ID is always located at the top_left of the ArUco marker
                cv2.putText(frame, str(marker_id),
                            (top_left[0], top_left[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

        # Display the resulting frame
        cv2.imshow('frame', frame)

        # If "q" is pressed on the keyboard,
        # exit this loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Close down the video stream
    cap.release()
    cv2.destroyAllWindows()

    #
    # def write_read(x):
    #     arduino.write(bytes(x, 'utf-8'))
    #     time.sleep(0.05)
    #     data = arduino.readline()
    #     return data
    # while True:
    #     num = input("Enter a number: ") # Taking input from user
    #     value = write_read(num)
    #     print(value)  # printing the value


    # ser.write(b'H')  # send the pyte string 'H'
    # time.sleep(0.5)  # wait 0.5 seconds
    # move.encode('utf-8')
    # arduino.write(move)
"""
