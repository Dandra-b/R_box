import rclpy, time
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from yolo_msgs.msg import Yolov8Inference
from std_msgs.msg import Int32

class ColorAndCircleDetection(Node):
    def __init__(self):
        super().__init__('color_and_circle_detection')
        self.get_logger().info("Puzzle catching lights.....")
        self.bridge = CvBridge()
        self.cv_image = None
        self.state = "start"
        #Declaraciones de subsripción y de publicador de imagen y velocidad del puzzlebot
        self.sub = self.create_subscription(Image, '/video_source/raw', self.camera_callback, 20)
        self.sub_inference = self.create_subscription(Yolov8Inference, "/yolov8_inference", self.traffic_signs_callback, 10)
        self.pub_error = self.create_subscription(Int32, "/error_center", self.center, 1)
        self.pub_center = self.create_subscription(Int32, "/no_center", self.no_center, 1)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
        self.state = "start" 
        #Declaraciones de las banderas de las señales de transito
        self.ahead = [0,0]
        self.left = [0,0]
        self.right = [0,0]
        self.stop = [0,0]
        #Declaración de la bandera del estado del semaforo
        self.semaforo_green = [0,0,0,0]
        self.class_name = None
        self.center_line = None
        self.nCenter = None
        self.msg = Twist()
        #self.publish_vel = self.create_publisher(Int)
        self.timer = self.create_timer(0.05, self.operations_callback)
    def center(self, msg):
        self.center_line = msg.data
    
    def no_center(self, msg):
        self.nCenter = msg.data

    def camera_callback(self, msg):
        #Recibimos la imagen capturada por el lente del puzzle y la convertimos para opencv
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def traffic_signs_callback(self, msg):
        for element in msg.yolov8_inference: 
            if element.class_name == "ahead_only":
                self.ahead[0] = 1
                area = (element.bottom - element.top)*(element.right - element.left)
                if area > 1600:
                    self.ahead[1] = 1
            if element.class_name == "turn_right_ahead":
                self.right[0] = 1
                area = (element.bottom - element.top)*(element.right - element.left)
                if area > 240:
                    self.right[1] = 1
                else:
                    self.right[1] = 0
            if element.class_name == "turn_left_ahead":
                self.left[0] = 1
                area = (element.bottom - element.top)*(element.right - element.left)
                if area > 2650:
                    self.left[1] = 1
                else:
                    self.left[1] = 0
            if element.class_name == "stop":
                self.stop[0] = 1
                area = (element.bottom - element.top)*(element.right - element.left)
                if area > 2650:
                    self.left[1] = 0
                else:
                    self.left[1] = 1
            if element.class_name == "red":
                self.semaforo_green[0] = 0
                self.semaforo_green[1] = 0
                self.semaforo_green[2] = 1
                self.semaforo_green[3] = 0
            if element.class_name == "green":
                self.semaforo_green[0] = 1
                self.semaforo_green[1] = 0
                self.semaforo_green[2] = 0
                area = (element.bottom - element.top)*(element.right - element.left)
                if area > 400:
                    self.semaforo_green[3] = 1
                else:
                    self.semaforo_green[3] = 0

    def idle(self):
        self.state = "follow_line"
    
    def linefollower(self):
        if self.center_line is not None:
            if (self.center_line > -17 and self.center_line < 17) and self.nCenter == 1:
                self.msg.linear.x = 0.03
                self.msg.angular.z = 0.0
                self.pub.publish(self.msg)
            elif self.center_line < -17 and self.nCenter == 1:
                self.msg.linear.x = 0.03
                self.msg.angular.z = -self.center_line*0.006
                self.pub.publish(self.msg)
            elif self.center_line > -17 and self.nCenter == 1:
                self.msg.linear.x = 0.03
                self.msg.angular.z = -self.center_line*0.006
                self.pub.publish(self.msg)
            elif self.nCenter == 0:
                self.state = "stop"
        # Dibujar contornos en la imagen original

    def stop_state(self):
        self.msg.linear.x = 0.0
        self.msg.angular.z = 0.0
        self.pub.publish(self.msg)
        self.state = "semaforo"

    #Estado donde hacemos la lectura del semaforo para así poder al estado de sign
    def semaforo(self):
        if self.semaforo_green[3] == 1 and self.semaforo_green[0] == 1:
            self.state = "sign" 
        else:
            self.state = "stop"
    def sign(self):
        if self.ahead[1] == 1:
            self.state = "ahead"
            
        if self.right[1] == 1:
            self.state = "right"

        if self.left[1] == 1:
            self.state = "left_turn"
            
    def go_ahead(self):
        print("Voy a acelerar")
        self.msg.linear.x = 0.075
        self.msg.angular.z = 0.0
        self.pub.publish(self.msg)
        time.sleep(7)
        self.ahead[1] = 0
        self.state = "follow_line"

    def right_turn(self):
        print("Voy a avanzar poquito")
        self.msg.linear.x = 0.015
        self.msg.angular.z = 0.0
        self.pub.publish(self.msg)
        time.sleep(1.5)
        print("Voy a dar vuelta a la derecha")
        self.msg.linear.x = 0.035   
        self.msg.angular.z = -0.07
        self.pub.publish(self.msg)
        time.sleep(4)
        self.right[1] = 0
        self.state = "follow_line"
    
    def turn_left(self):
        print("Voy a avanzar poquito")
        self.msg.linear.x = 0.015
        self.msg.angular.z = 0.0
        self.pub.publish(self.msg)
        time.sleep(1.5)
        print("Voy a dar vuelta a la izquierda")
        self.msg.linear.x = 0.035   
        self.msg.angular.z = 0.07
        self.pub.publish(self.msg)
        time.sleep(4)
        self.left[1] = 0
        self.state = "follow_line"
        


    def operations_callback(self):
        if self.cv_image is not None:
            if self.state == "start":
                self.idle()
                print("sali de start")
            elif self.state == "follow_line":
                self.linefollower()
                print("sali de line")
            elif self.state == "stop":
                self.stop_state()
                print("sali de stop")
            elif self.state == "semaforo":
                self.semaforo()
                print("Saliendo de semaforo")
            elif self.state == "sign":
                self.t0 = time.time()
                self.elapsed_time = time.time()-self.t0
                self.sign()
                if self.state == "ahead":
                    self.go_ahead()
                elif self.state == "right":
                    self.right_turn()
                    
def main(args=None):
    rclpy.init(args=args)
    nodeh = ColorAndCircleDetection()
    try: rclpy.spin(nodeh)
    except Exception as error: print(error)
    except KeyboardInterrupt: print("Node interrupted")

if __name__ == '__main__':
    main()
