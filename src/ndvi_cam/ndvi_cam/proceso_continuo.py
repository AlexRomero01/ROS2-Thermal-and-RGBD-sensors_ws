import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  
from std_msgs.msg import Float32MultiArray 
from ultralytics_ros.msg import YoloResult
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import pandas as pd
import os
from datetime import datetime


class ContinuousProcessNode(Node):
    #initialization
    def __init__(self):
        super().__init__('continuous_process_node')

        #subscriptions are initialized
        self.subscriptionNDVI = self.create_subscription(String, '/NDVI',self.listener_NDVI,10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.listener_callback,
            10)
        self.color_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_listener_callback,
            10)
        
        # Publishers for the depth and color images
        self.depth_image_publisher = self.create_publisher(Image, '/depth_image', 10)
        self.color_image_publisher = self.create_publisher(Image, '/color_image', 10)
        
        
        # #calibration point variables are started with the center pixel as default
        # self.lista_posicion = None
        # self.x_calibracion= 640    # 627
        # self.y_calibracion= 360    # 299

        # #calibration point reading
        # with open('position.txt', 'r') as file:     # this txt file is considered to be at /home/user
        #     content = file.read()
        # value1, value2 = content.split()
        # file.close()

        # self.x_calibracion=int(value1)
        # self.y_calibracion=int(value2)
        # self.get_logger().info('Calibration pixel: ('+str(self.x_calibracion)+','+str(self.y_calibracion)+')')

        self.csv_directory = '/home/yeray/sensors_ws_tot/src/Mediciones/Continuo'  # Cambia esto a la ruta deseada
        os.makedirs(self.csv_directory, exist_ok=True)  # Crea la carpeta si no existe

        #initialize CSV file if it does not exist to save NDVI
        now=datetime.now().strftime('%d-%m-%Y, %H:%M:%S')
        self.csv_file = os.path.join(self.csv_directory, f'Mediciones_NDVI_continuo_{now}.csv')
        if not os.path.isfile(self.csv_file):
            df = pd.DataFrame(columns=['NDVI_value'])
            df.to_csv(self.csv_file, index=False)
        
        #create a timer to save NDVI values ​​every second
        self.timer = self.create_timer(1.0, self.save_ndvi_value)

        #variable initialization that indicates whether information is received from the sensor node and the NDVI value
        self.funcionaNDVI=None
        self.NDVI_value = None
        #subscriptions are launched again to prevent them from failing
        self.subscriptionNDVI
        self.subscription 
        self.color_subscription  

        #images are initialized
        self.bridge = CvBridge()
        self.bridge_color = CvBridge()

        #other necessary variables are initialized
        self.center_depth =None
        self.punto_calibra = None
        self.lista_posición=None
        self.distancia_es_correcta=True
        self.error_distancia=False
        self.texto_distancia=''



    #NDVI sensor node subscriber function
    def listener_NDVI(self,data):
        self.NDVI_value= data
        if self.funcionaNDVI == None:
            self.get_logger().info('NDVI works')
            self.funcionaNDVI=1
        if self.distancia_es_correcta == True:
            self.save_ndvi_value()
 
    #depth image subscriber function
    def listener_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        self.x_calibracion_depth = cv_image.shape[1]//2
        self.y_calibracion_depth = cv_image.shape[0]//2
        # self.get_logger().warn(f"cv_image.shape[0]: {cv_image.shape[0]}") # 480
        # self.get_logger().warn(f"cv_image.shape[1]: {cv_image.shape[1]}") # 848
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(cv_image, alpha=0.03), cv2.COLORMAP_JET)
        
        #calculates the depth at the calibration point and with it indicates to the robot whether it should move closer or further away
        self.center_depth = cv_image[int(self.x_calibracion_depth), int(self.y_calibracion_depth)] 
        center_depth =float(self.center_depth)

        if center_depth < 762 and center_depth > 0 and (self.distancia_es_correcta == True or self.error_distancia==True):
            self.get_logger().info('Get away')
            self.texto_distancia='Get away'
            self.distancia_es_correcta=False
            self.error_distancia=False
        elif center_depth > 915  and (self.distancia_es_correcta == True or self.error_distancia==True):
            self.get_logger().info('Come closer')
            self.texto_distancia='Come closer'
            self.distancia_es_correcta=False
            self.error_distancia=False
        elif center_depth == 0 and self.error_distancia == False:
            self.get_logger().info('Distance error')
            self.texto_distancia='Distance error'
            self.distancia_es_correcta=False
            self.error_distancia=True
        elif center_depth < 915 and center_depth > 762:
            if self.distancia_es_correcta == False or self.error_distancia==True:
                self.texto_distancia='Correct measuring distance'
                self.get_logger().info('Correct measuring distance')
                self.distancia_es_correcta=True
                self.error_distancia=False
        

        #marks the calibration point
        marked_image = cv2.circle(depth_colormap, (self.x_calibracion_depth, self.y_calibracion_depth), 5, (0,0,255), -1)
        # self.get_logger().warn(f"marked_image.shape[0]: {marked_image.shape[0]}") # 480
        # self.get_logger().warn(f"marked_image.shape[1]: {marked_image.shape[1]}") # 848
        
        #approximate measurement zone line mark
        line_length = int(cv_image.shape[1]//2) #based on the FOV of each element and the measurement distance
        start_point = (self.x_calibracion_depth - line_length//2, self.y_calibracion_depth)
        end_point = (self.x_calibracion_depth + line_length//2, self.y_calibracion_depth)

        marked_image = cv2.line(marked_image, start_point, end_point, (0,0,255), 2)
        
        #write the NDVI and the distance above the line
        text_position = (start_point[0], start_point[1] - 10)
        if self.NDVI_value is not None:
            text_NDVI = 'Distance:'+ str(center_depth) + 'mm, NDVI: ' + str(self.NDVI_value)[25:32]
        else: 
            text_NDVI = 'Distance:'+ str(center_depth) + 'mm'+'NDVI: no information'
        cv2.putText(marked_image, text_NDVI, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        #add motion cues to the bottom left of the image
        donde_posicion = (10, marked_image.shape[0] - 10)
        donde=self.texto_distancia
        cv2.putText(marked_image, donde, donde_posicion, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        # Convert the processed image back to ROS Image message and publish it
        try:
            depth_image_msg = self.bridge.cv2_to_imgmsg(marked_image, encoding="bgr8")
            self.depth_image_publisher.publish(depth_image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting depth image: {e}")
        

        #show image in a tab
        cv2.imshow('RealSense Depth Image', marked_image)
        cv2.waitKey(1)



    #RGB image subscriber function
    def color_listener_callback(self, data):
        cv_image2 = self.bridge_color.imgmsg_to_cv2(data, desired_encoding='bgr8')
        # self.get_logger().warn(f"cv_image2.shape[0]: {cv_image2.shape[0]}") # 720
        # self.get_logger().warn(f"cv_image2.shape[1]: {cv_image2.shape[1]}") # 1280
        self.x_calibracion_rgb = cv_image2.shape[1]//2
        self.y_calibracion_rgb = cv_image2.shape[0]//2
        #marks the calibration point
        marked_image2 = cv2.circle(cv_image2, (self.x_calibracion_rgb, self.y_calibracion_rgb), 5, (0,0,255), -1)
        # self.get_logger().warn(f"color_image.shape[0]: {marked_image2.shape[0]}") # 720
        # self.get_logger().warn(f"color_image.shape[1]: {marked_image2.shape[1]}") # 1280
        
        #approximate measurement zone line mark
        line_length = int(cv_image2.shape[1]//2)  #based on the FOV of each element and the measurement distance
        start_point = (self.x_calibracion_rgb - line_length//2, self.y_calibracion_rgb)
        end_point = (self.x_calibracion_rgb + line_length//2, self.y_calibracion_rgb)
        marked_image2 = cv2.line(marked_image2, start_point, end_point, (0,0,255), 2)

        #write the NDVI and the distance above the line
        text_position = (start_point[0], start_point[1] - 10)
        if self.NDVI_value is not None:
            text_NDVI = 'Distance:'+ str(self.center_depth) + 'mm, NDVI: ' + str(self.NDVI_value)[25:32]
        else: 
            text_NDVI = 'Distance:'+ str(self.center_depth) + 'mm'+ 'NDVI: no information'
        cv2.putText(marked_image2, text_NDVI, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        #add motion cues to the bottom left of the image
        donde_posicion = (10, marked_image2.shape[0] - 10)  #margin: (x,y)-10 px
        donde=self.texto_distancia
        cv2.putText(marked_image2, donde, donde_posicion, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Convert the processed image back to ROS Image message and publish it
        try:
            color_image_msg = self.bridge_color.cv2_to_imgmsg(marked_image2, encoding="bgr8")
            self.color_image_publisher.publish(color_image_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Error converting color image: {e}")
                
        #show image in a tab
        cv2.imshow('RealSense Color Image', marked_image2)
        cv2.waitKey(1)



    #function that saves the NDVI value to a CSV file
    def save_ndvi_value(self):
        if self.NDVI_value is not None:
            ndvi_value = float(str(self.NDVI_value)[26:32])
            
            #reads the current CSV file, adds the new value and saves it
            try:
                df = pd.read_csv(self.csv_file)
            except FileNotFoundError:
                #if the file does not exist, a new one is created with the 'NDVI_value' column
                df = pd.DataFrame(columns=['NDVI_value'])
        
            if pd.notna(ndvi_value):  # Ensure the value is not NA
                df = df._append({'NDVI_value': ndvi_value}, ignore_index=True)
            df.to_csv(self.csv_file, index=False)
        

#main function that activates the node
def main(args=None):
    rclpy.init(args=args)
    continuous_porcess_node = ContinuousProcessNode()
    rclpy.spin(continuous_porcess_node)
    continuous_porcess_node.destroy_node()
    rclpy.shutdown()

#if the main function is called the node is activated
if __name__ == '__main__':
    main()

