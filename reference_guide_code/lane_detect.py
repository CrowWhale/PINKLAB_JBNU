import rclpy

import time
# from time import time, sleep, strftime
from rclpy.node import Node
import threading
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from rclpy.duration import Duration
from cv_bridge import CvBridge, CvBridgeError

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2

from datetime import datetime, timedelta



class LaneDetect(Node):
    def __init__(self):
        super().__init__('lane_detect')

        
        self.bridge = CvBridge()

        # yellow lines subscribe
        # Pinkbot의 PiCamera로부터 CompressedImage를 전송받기 위한 ROS subscription 설정 
        self.subscription = self.create_subscription(
            CompressedImage, # 수신할 메시지 타입 지정
            'image_raw/compressed', # 구독할 토픽의 이름 지정
            self.y_CvImage, # 메시지가 도착했을때 실행할 callback 함수 지정
            10 ) # 메세지를 저장하는 버퍼의 크기 설정
        self.subscription  # prevent unused variable warning

        # # white lines subscribe
        # # Pinkbot의 PiCamera로부터 CompressedImage를 전송받기 위한 ROS subscription 설정 
        # self.subscription = self.create_subscription(
        #     CompressedImage, # 수신할 메시지 타입 지정
        #     'image_raw/compressed', # 구독할 토픽의 이름 지정
        #     self.w_CvImage, # 메시지가 도착했을때 실행할 callback 함수 지정
        #     10 ) # 메세지를 저장하는 버퍼의 크기 설정
        # self.subscription  # prevent unused variable warning
        
        
        self.get_logger().info("Ready to detecting!")
            
    def y_CvImage(self,msg):

        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            print("Oops, CvBridge isn't working!")
        
        # 영상에서 특정 영역의 색상만 추출시킨다.
        def bird(img):
            imshape = img.shape

            # width = imshape[1]
            height = imshape[0]

            pts1 = np.float32([[60, 200],[60, height],[340,200],[340, height]]) 

            # cv2.circle(img, [60, 200], 5, (255,0,0),-1)
            # cv2.circle(img, [60, height], 5, (0,255,0),-1)
            # cv2.circle(img, [340,200], 5, (0,0,255),-1)
            # cv2.circle(img, [340, height], 5, (0,0,0),-1)

            # 좌표의 이동점
            pts2 = np.float32([[10,10],[10,1000],[1000,10],[1000,1000]])

            M = cv2.getPerspectiveTransform(pts1, pts2)

            img = cv2.warpPerspective(img, M, (1100,1000))

            b, g, r = cv2.split(img)

            new_img = cv2.merge((b,g,r))

            return new_img

        new_img = bird(img)


        def remake(img):
            dst1 = cv2.inRange(img, (0,160,160), (150, 255, 255))
            # dst1 = cv2.resize(dst1, (960,540))
            return dst1
    
        edges = remake(new_img)

        def region_of_interest(img, vertices):

            mask = np.zeros_like(img)

            if len(img.shape) > 2:
                channel_count = img.shape[2]
                ignore_mask_color = (255,) * channel_count
            else:
                ignore_mask_color = 255

            cv2.fillPoly(mask, vertices, ignore_mask_color)

            masked_image = cv2.bitwise_and(img, mask)
            
            return masked_image

        new_imshape = new_img.shape

        height = new_imshape[0]
        width = new_imshape[1]

        vertices = np.array([[(width/4, height),
                                (width/4,0),
                                (width*3/4,0),
                                (width*3/4, height)]], dtype=np.int32)

        mask = region_of_interest(edges, vertices)

        def draw_lines(img, lines, color=[0, 255, 255], thickness=5):
            avg_x = []
            avg_y = []

            if lines is not None:

                for line in lines:
                    for x1, y1, x2, y2 in line:
                        cv2.line(img, (x1, y1), (x2, y2), color, thickness)
                        avg_x_val = (x1+x2)/2
                        avg_y_val = (y1+y2)/2
                        avg_x.append(avg_x_val)
                        avg_y.append(avg_y_val)
            
                total_avg_x = sum(avg_x) / len(avg_x) 
                total_avg_y = sum(avg_y) / len(avg_y) 

                with open('state.txt', 'r') as f:
                    state = f.read()
                state = int(state)
                

                # Go
                if (width/2 - 50) <= total_avg_x <= (width/2 + 50):
                    
                    if (state == 1):
                        pass
                    else:
                        with open('state.txt', 'w') as f:
                            f.write('1')     

                    
        def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
            lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
            line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
            draw_lines(line_img, lines)

            return line_img

        rho = 2
        theta = np.pi/180
        threshold = 90
        min_line_len = 120
        max_line_gap = 150

        lines = hough_lines(mask, rho, theta, threshold, min_line_len, max_line_gap)

        cv2.imshow('y_image_raw/compressed', lines)
        cv2.imshow('original', img)
        cv2.waitKey(1)
    
    def w_CvImage(self,msg):

        try:
            img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError:
            print("Oops, CvBridge isn't working!")
        
        # 영상에서 특정 영역의 색상만 추출시킨다.
        def bird(img):
            imshape = img.shape

            width = imshape[1] # ㅡ
            height = imshape[0] #  |

            pts1 = np.float32([[200, 200],[200, height],[width-200,200],[width-200, height]]) 

            # 좌표의 이동점
            pts2 = np.float32([[10,10],[10,1000],[1000,10],[1000,1000]])

            M = cv2.getPerspectiveTransform(pts1, pts2)

            img = cv2.warpPerspective(img, M, (1100,1000))

            b, g, r = cv2.split(img)

            new_img = cv2.merge((b,g,r))

            return new_img

        new_img = bird(img)


        def remake(img):
            dst1 = cv2.inRange(img, (200,200,200), (255, 255, 255))
            # dst1 = cv2.resize(dst1, (960,540))
            return dst1
    
        edges = remake(new_img)

        def region_of_interest(img, vertices):

            mask = np.zeros_like(img)

            if len(img.shape) > 2:
                channel_count = img.shape[2]
                ignore_mask_color = (255,) * channel_count
            else:
                ignore_mask_color = 255

            cv2.fillPoly(mask, vertices, ignore_mask_color)

            masked_image = cv2.bitwise_and(img, mask)
            
            return masked_image

        new_imshape = new_img.shape

        height = new_imshape[0]
        width = new_imshape[1]

        vertices = np.array([[(width/4, height),
                                (width/4,0),
                                (width*3/4,0),
                                (width*3/4, height)]], dtype=np.int32)

        mask = region_of_interest(edges, vertices)


        def draw_lines(img, lines, color=[255, 255, 255], thickness=5):
            avg_x = []
            avg_y = []
            
            if lines is not None:
        
                for line in lines:
                    for x1, y1, x2, y2 in line:
                        cv2.line(img, (x1, y1), (x2, y2), color, thickness)
                        avg_x_val = (x1+x2)/2
                        avg_y_val = (y1+y2)/2
                        avg_x.append(avg_x_val)
                        avg_y.append(avg_y_val)
            
                total_avg_x = sum(avg_x) / len(avg_x) 
                total_avg_y = sum(avg_y) / len(avg_y) 

        def hough_lines(img, rho, theta, threshold, min_line_len, max_line_gap):
            lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
            line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
            draw_lines(line_img, lines)

            return line_img

        rho = 2
        theta = np.pi/180
        threshold = 90
        min_line_len = 120
        max_line_gap = 150

        lines = hough_lines(mask, rho, theta, threshold, min_line_len, max_line_gap)


        # def weighted_img(img, initial_img, a=0.8, b=1., c=0.):
        #     return cv2.addWeighted(initial_img, a, img, b,c)

        # lines_edges = weighted_img(lines,new_img, a=0.8, b=1., c=0.)
        cv2.imshow('w_image_raw/compressed', lines)
        cv2.imshow('original', img)
        cv2.waitKey(1)

            
            

def main(args=None):
    rclpy.init(args=args)
    
    lane_detect = LaneDetect()
    rclpy.spin(lane_detect)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    