
import rclpy

import time
from rclpy.node import Node
import threading
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from rclpy.duration import Duration
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
 
import sys

class ArucoDetect(Node):
    def __init__(self):
        super().__init__('aruco_detect')
        
        self.ARUCO_DICT= {
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
                    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
                    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
                    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
                    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
                    }
        
    def aruco_topic():
        self.aruco_subscriber = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            self.CvImage,
            10 )

        self.aruco_publisher = self.create_publisher(
            Int32MultiArray,
            '/detect_aruco_num',
            10
        )  
 
        self.bridge = CvBridge()

    def CvImage(self, msg):
        try:
            self.image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            print("CvBridge is not working, check plz")


        def aruco_display(self, corners, ids, rejected, image):
            
            if len(corners) > 0:
                # flatten the ArUco IDs list
                ids = ids.flatten()
                # loop over the detected ArUCo corners
                for (markerCorner, markerID) in zip(corners, ids):
                    # extract the marker corners (which are always returned in
                    # top-left, top-right, bottom-right, and bottom-left order)
                    corners = markerCorner.reshape((4, 2))
                    (topLeft, topRight, bottomRight, bottomLeft) = corners
                    # convert each of the (x, y)-coordinate pairs to integers
                    topRight = (int(topRight[0]), int(topRight[1]))
                    bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    topLeft = (int(topLeft[0]), int(topLeft[1]))

                    cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                    cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                    cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                    cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                    # compute and draw the center (x, y)-coordinates of the ArUco
                    # marker
                    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                    cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
                    # draw the ArUco marker ID on the image
                    cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)
                    print("[Inference] ArUco marker ID: {}".format(markerID))
                    # show the output image
            return image

        def aruco_detect(self, camera, video, aruco_type):
            if self.ARUCO_DICT.get(aruco_type, None) is None :
                print(f"ArUCo tag type '{aruco_type}' is not supproted")
                sys.exit(0)

            aruco_dict = cv2.aruco.Dictionary_get(self.ARUCO_DICT[aruco_type])
            aruco_params = cv2.aruco.DetectorParameters_create()

            h, w, _ = self.image.shape

            width = 1000
            height = int(width * (h / w))
            frame = cv2.resize(self.image, (width, height), interpolation=cv2.INTER_CUBIC)
            corners, ids, rejected = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=aruco_params)

            detected_markers = aruco_display(self, corners, ids, rejected, frame)

            return detected_markers
        
        last_iamge = aruco_detect(self, camera=True, video=None, aruco_type="DICT_4X4_50")

        cv2.imshow("image_raw/compressed", last_iamge)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)
    
    aruco_detect = ArucoDetect()
    rclpy.spin(aruco_detect)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    


