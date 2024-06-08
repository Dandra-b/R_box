import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorAndCircleDetection(Node):
    def __init__(self):
        super().__init__('color_and_circle_detection')
        self.get_logger().info("Puzzle catching lights.....")
        self.bridge = CvBridge()
        self.cv_image = None
        #Declaraciones de subsripción y de publicador de imagen y velocidad del puzzlebot
        self.sub = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 20)
        self.pub_error = self.create_publisher(Int32, "/error_center", 1)
        self.pub_center = self.create_publisher(Int32, "/no_center", 1)
        #self.publish_vel = self.create_publisher(Int)
        self.timer = self.create_timer(0.05, self.operations_callback)
        
    def camera_callback(self, msg):
        #Recibimos la imagen capturada por el lente del puzzle y la convertimos para opencv
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
    def operations_callback(self):
        if self.cv_image is not None:
            msg = Int32()
            msg_error = Int32()
            height, width = self.cv_image.shape[:2]
            # Define the region of interest (lower fourth of the frame)
            lower_fourth = self.cv_image[height * 3 // 4:height, 0:width]
            roi_height, roi_width = lower_fourth.shape[:2]
            margin = 30
            # Ensure the margin does not exceed half the width
            margin = min(margin, roi_width // 2)
            roi = lower_fourth[0:roi_height // 2, margin:roi_width - margin]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            lower_black = np.array([0,0,0])   # Rango mínimo de color negro
            #upper_black = np.array([179,147,90]) #Rango máximo de color negro
            upper_black = np.array([180,110,59]) 
            black_mask = cv2.inRange(hsv, lower_black, upper_black)
            black_mask = cv2.GaussianBlur(black_mask, (5,5), 0) #intento1
            #black_mask = cv2.blur(black_mask,(5,5)) #intento2
            # Encontrar contornos en la imagen
            contours, _ = cv2.findContours(black_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if len(contours) > 0:
                
                c = max(contours,key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] != 0:
                    print("ando encontrando linea")
                    msg_error.data = 1
                    self.pub_center.publish(msg_error)
                    cx = int(M["m10"]/M["m00"])
                    cy = int(M["m01"]/M["m00"])
                    error_x = cx-90
                    self.get_logger().info(str(error_x))
                    msg.data = error_x
                    self.pub_error.publish(msg)

                    cv2.circle(roi,(cx,cy), 5, (0,255,255), -1)
                    # Dibujar contornos en la imagen original
            else:
                print("NO ENCONTRE LINEA")
                msg_error.data = 0
                self.pub_center.publish(msg_error)
            cv2.drawContours(roi, contours, -1, (0, 255, 0), 2)
        # Mostrar la imagen con los contornos
            cv2.imshow('Contours', roi)
            cv2.waitKey(1)
                    
def main(args=None):
    rclpy.init(args=args)
    nodeh = ColorAndCircleDetection()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node interrupted")

if __name__ == '__main__':
    main()
