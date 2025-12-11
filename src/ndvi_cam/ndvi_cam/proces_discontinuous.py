import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  
from std_msgs.msg import Float32MultiArray 
from ultralytics_ros.msg import YoloResult
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import ast
import pandas as pd
import os
from datetime import datetime
import time

class DisontinuousProcessNode(Node):
    #initialization
    def __init__(self):
        super().__init__('discontinuous_process_node')

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
        
        self.detec_subscription = self.create_subscription(
            YoloResult,
            '/yolo_result',
            self.detection_callback,
            10) 
        
        # Publishers for the depth and color images
        self.color_image_publisher = self.create_publisher(Image, '/color_image', 10)
        

        # #calibration point variables are started with the center pixel as default
        # self.lista_posicion = None
        # self.x_calibracion=640
        # self.y_calibracion=360

        # #calibration point reading
        # with open('position.txt', 'r') as file:
        #     content = file.read()
        # value1, value2 = content.split()
        # file.close()

        # self.x_calibracion=int(value1)
        # self.y_calibracion=int(value2)
        # self.get_logger().info('Calibration pixel: ('+str(self.x_calibracion)+','+str(self.y_calibracion)+')')


        #subscriptions are launched again to prevent them from failing
        self.detec_subscription
        self.subscriptionNDVI
        self.subscription 
        self.color_subscription 

        #images are initialized
        self.bridge = CvBridge()
        self.bridge_color = CvBridge()

        #variable initialization that indicates whether information is received from the sensor node and the NDVI value
        self.funcionaNDVI=None
        self.NDVI_value = None

        #initialization of variables for movement
        self.lista_x=None
        self.lista_y=None
        self.lista_box_x=None
        self.lista_box_y=None
        self.ha_tomado_medida=False
        self.donde=''
        self.distancia_es_correcta=True
        self.error_distancia=False
        self.texto_distancia=''
        
        #other necessary variables are initialized
        self.center_depth =None
        self.punto_calibra = None
        self.lista_posición=None

        self.csv_directory = '/home/yeray/sensors_ws_tot/src/Mediciones/Discontinuo'  # Cambia esto a la ruta deseada
        os.makedirs(self.csv_directory, exist_ok=True)  # Crea la carpeta si no existe

        #initialize CSV file if it does not exist to save NDVI
        now=datetime.now().strftime('%d-%m-%Y, %H:%M:%S')
        self.csv_file = os.path.join(self.csv_directory, f'Mediciones_NDVI_discontinuo_{now}.csv')
        if not os.path.isfile(self.csv_file):
            df = pd.DataFrame(columns=['NDVI_value'])
            df.to_csv(self.csv_file, index=False)
    
    #YOLO node subscriber function
    def detection_callback(self, msg):
        #create a list with the position of the detected objects
        self.lista_x=[]
        self.lista_y=[]
        self.lista_box_x=[]
        self.lista_box_y=[]
        for detection in msg.detections.detections:
            if detection.results:
                #the first result is taken as the most probable if there are several
                result = detection.results[0]
                class_id = result.hypothesis.class_id
                score = result.hypothesis.score
                center_x = detection.bbox.center.position.x
                center_y = detection.bbox.center.position.y
                ancho_x = int(detection.bbox.size_x)
                alto_y = int(detection.bbox.size_y)
                self.lista_x.append(center_x)
                self.lista_y.append(center_y)
                self.lista_box_x.append(ancho_x)
                self.lista_box_y.append(alto_y)
                #prints the center coordinates and name of the detected object
                self.get_logger().info(f'Detected object: {class_id} with trust {score:.2f} in ({center_x}, {center_y})')
            else:
                self.get_logger().info('No objects were detected in this image.')
        

        #movement of the robot to place it in the best position to make the measurement
        #tolerance regarding calibration position
        num_pixels_coloc_x= 20
        num_pixels_coloc_y=40

        index_next_value=-1
        min_y=800000

        for i, elem in enumerate(self.lista_y):
            if elem < min_y and elem > self.y_calibracion_rgb + num_pixels_coloc_y:
                index_next_value=i
                min_y=elem

        if len(self.lista_x) == 0: #does not detect anything
            self.donde='No object detected move forward'
            self.ha_tomado_medida=False
        else: #detects something
            center_x=self.lista_x[index_next_value]
            center_y=self.lista_y[index_next_value]
            num_pixels_coloc_x= self.lista_box_x[index_next_value]//5
            num_pixels_coloc_y=self.lista_box_y[index_next_value]//5

            if center_y > self.y_calibracion_rgb-num_pixels_coloc_y: #has not yet been calculated
                if center_x > self.x_calibracion_rgb+num_pixels_coloc_x:
                    self.donde='Move to the left'
                    self.ha_tomado_medida=False
                    if center_y > self.y_calibracion_rgb+num_pixels_coloc_y:
                        self.donde=self.donde + ' and ahead'   

                elif center_x < self.x_calibracion_rgb-num_pixels_coloc_x:
                    self.donde='Move to the right'
                    self.ha_tomado_medida=False
                    if center_y > self.y_calibracion_rgb+num_pixels_coloc_y:
                        self.donde=self.donde + ' and ahead'
                elif center_x > self.x_calibracion_rgb-num_pixels_coloc_x and center_x < self.x_calibracion_rgb+num_pixels_coloc_x and center_y > self.y_calibracion_rgb+num_pixels_coloc_y:
                    self.donde='In the center, move forward'
            
                elif  center_y < self.y_calibracion_rgb+num_pixels_coloc_y and center_y > self.y_calibracion_rgb-num_pixels_coloc_y and center_x > self.x_calibracion_rgb-num_pixels_coloc_x and center_x < self.x_calibracion_rgb+num_pixels_coloc_x:
                    if self.ha_tomado_medida == False:
                        if self.distancia_es_correcta==True:
                            self.donde='Focused, taking measurements'
                            self.save_ndvi_value()
                            self.ha_tomado_medida=True
                        else:
                            self.donde='Focused, the distance is not correct'
                    else:
                        self.donde='Move forward just a little'
            else: #what is detected has already been calculated and moved on to the next
                self.donde='Move forward'


        self.get_logger().info(self.donde)


    #NDVI sensor node subscriber function
    def listener_NDVI(self,data):
        self.NDVI_value= data
        if self.funcionaNDVI == None:
            self.get_logger().info('NDVI works')
            self.funcionaNDVI=1
 
    #depth image subscriber function
    def listener_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(cv_image, alpha=0.03), cv2.COLORMAP_JET)
        self.x_calibracion_depth = cv_image.shape[1]//2
        self.y_calibracion_depth = cv_image.shape[0]//2
        
        #calculates the depth at the calibration point and with it indicates to the robot whether it should move closer or further away
        self.center_depth = cv_image[self.x_calibracion_depth, self.y_calibracion_depth] 
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

    #RGB image subscriber function
    def color_listener_callback(self, data):
        cv_image2 = self.bridge_color.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.x_calibracion_rgb = cv_image2.shape[1]//2
        self.y_calibracion_rgb = cv_image2.shape[0]//2
        
        #marks the calibration point
        marked_image2 = cv2.circle(cv_image2, (self.x_calibracion_rgb, self.y_calibracion_rgb), 5, (0,0,255), -1)
        
        #approximate measurement zone line mark
        line_length = int(cv_image2.shape[1]//2)   #based on the FOV of each element and the measurement distance
        start_point = (self.x_calibracion_rgb - line_length//2, self.y_calibracion_rgb)
        end_point = (self.x_calibracion_rgb + line_length//2, self.y_calibracion_rgb)
        marked_image2 = cv2.line(marked_image2, start_point, end_point, (0,0,255), 2)

        #write the NDVI and the distance above the line
        text_position = (start_point[0], start_point[1] - 10)
        if self.NDVI_value is not None:
            text_NDVI = 'Distance:'+ str(self.center_depth) + 'mm, NDVI: ' + str(self.NDVI_value)[25:32]
        else: 
            text_NDVI = 'Distance:'+ str(self.center_depth) +'mm, NDVI: no information'
        cv2.putText(marked_image2, text_NDVI, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)

        #add motion cues to the bottom left of the image
        donde_posicion = (10, marked_image2.shape[0] - 10)
        donde=self.donde+'. '
        cv2.putText(marked_image2, donde, donde_posicion, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        dist_posicion = (10, marked_image2.shape[0] - 40)
        texto_distan=self.texto_distancia
        cv2.putText(marked_image2, texto_distan, dist_posicion, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

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
        
            df = df._append({'NDVI_value': ndvi_value}, ignore_index=True)
            df.to_csv(self.csv_file, index=False)
            
#main function that activates the node
def main(args=None):
    rclpy.init(args=args)
    discontinuosu_proces_node = DisontinuousProcessNode()
    rclpy.spin(discontinuosu_proces_node)
    discontinuosu_proces_node.destroy_node()
    rclpy.shutdown()

#if the main function is called the node is activated
if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from std_msgs.msg import String  
# from std_msgs.msg import Float32MultiArray 
# from ultralytics_ros.msg import YoloResult
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import ast
# import pandas as pd
# import os
# from datetime import datetime
# import time

# class RealSenseDepthNode(Node):
#     def __init__(self):
#         super().__init__('realsense_depth_node')
#         self.subscriptionNDVI = self.create_subscription(String, '/NDVI',self.listener_NDVI,10)
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera/camera/depth/image_rect_raw',
#             self.listener_callback,
#             10)
#         self.color_subscription = self.create_subscription(
#             Image,
#             '/camera/camera/color/image_raw',
#             self.color_listener_callback,
#             10)
        
#         self.detec_subscription = self.create_subscription(
#             YoloResult,
#             '/yolo_result',
#             self.detection_callback,
#             10) 
        
#         self.lista_posicion = None
#         self.x_calibracion=640
#         self.y_calibracion=360
#         self.donde=None


#         #lectura punto calibracion
#         with open('position.txt', 'r') as file:
#             content = file.read()
#         value1, value2 = content.split()
#         file.close()

#         self.funcionaNDVI=None

#         self.x_calibracion=int(value1)
#         self.y_calibracion=int(value2)
#         self.get_logger().info('Calibration pixel: ('+str(self.x_calibracion)+','+str(self.y_calibracion)+')')

        
#         self.detec_subscription
#         self.subscriptionNDVI
#         self.subscription 
#         self.color_subscription  # prevent unused variable warning
#         self.bridge = CvBridge()
#         self.NDVI_value = None
#         self.bridge_color = CvBridge()
#         self.center_depth =None
#         self.punto_calibra = None
#         self.lista_posición=None

#         #varibles para movimiento
#         self.lista_x=None
#         self.lista_y=None
#         self.ha_tomado_medida=False

#         # Inicializar el archivo CSV si no existe para guardar NDVI
#         now=datetime.now().strftime('%d-%m-%Y, %H:%M:%S')
#         self.csv_file = 'Mediciones_NDVI_discontinuo'+str(now)+'.csv'
#         if not os.path.isfile(self.csv_file):
#             df = pd.DataFrame(columns=['NDVI_value'])
#             df.to_csv(self.csv_file, index=False)
    
#     def detection_callback(self, msg):
#         self.lista_x=[]
#         self.lista_y=[]
#         for detection in msg.detections.detections:
#             if detection.results:
#                 # Tomamos el primer resultado como el más probable si hay varios
#                 result = detection.results[0]
#                 class_id = result.hypothesis.class_id
#                 score = result.hypothesis.score
#                 center_x = detection.bbox.center.position.x
#                 center_y = detection.bbox.center.position.y
#                 self.lista_x.append(center_x)
#                 self.lista_y.append(center_y)

#                 # Imprime las coordenadas del centro y el nombre del objeto detectado
#                 self.get_logger().info(f'Objeto detectado: {class_id} con confianza {score:.2f} en ({center_x}, {center_y})')
#             else:
#                 self.get_logger().info('No se detectaron objetos en esta imagen.')
        
#         num_pixels_coloc_x= 20
#         num_pixels_coloc_y=40

#         index_next_value=-1
#         min_y=800000

#         for i, elem in enumerate(self.lista_y):
#             if elem < min_y and elem > self.y_calibracion + num_pixels_coloc_y:
#                 index_next_value=i
#                 min_y=elem


#         if len(self.lista_x) == 0: #no detecta nada
#             self.donde='Muevete hacia delante'
#             self.ha_tomado_medida=False
#         else: #detecta algo
#             center_x=self.lista_x[index_next_value]
#             center_y=self.lista_y[index_next_value]
#             if center_y > self.y_calibracion-num_pixels_coloc_y: #no se ha calculado
#                 if center_x > self.x_calibracion+num_pixels_coloc_x:
#                     self.donde='Muevete hacia la derecha'
#                     self.ha_tomado_medida=False
#                     if center_y > self.y_calibracion+num_pixels_coloc_y:
#                         self.donde=self.donde + ' y delante'   

#                 elif center_x < self.x_calibracion-num_pixels_coloc_x:
#                     self.donde='Muevete hacia la izquierda'
#                     self.ha_tomado_medida=False
#                     if center_y > self.y_calibracion+num_pixels_coloc_y:
#                         self.donde=self.donde + ' y delante'
#                 elif center_x > self.x_calibracion-num_pixels_coloc_x and center_x < self.x_calibracion+num_pixels_coloc_x and center_y > self.y_calibracion+num_pixels_coloc_y:
#                     self.donde='Centrado muevete hacia delante'
            
#                 elif  center_y < self.y_calibracion+num_pixels_coloc_y and center_y > self.y_calibracion-num_pixels_coloc_y and center_x > self.x_calibracion-num_pixels_coloc_x and center_x < self.x_calibracion+num_pixels_coloc_x:
#                     if self.ha_tomado_medida == False:
#                         self.donde='Centrado,tomando medidas'
#                         self.save_ndvi_value()
#                         self.ha_tomado_medida=True
#                     else:
#                         self.donde='Muevete hacia delante solo un poco'
#             else:#se ha calculado lo que se detecta y se pasa a siguiente
#                 self.donde='Muevete hacia delante'


#         self.get_logger().info(self.donde)



#     def listener_NDVI(self,data):
#         self.NDVI_value= data
#         if self.funcionaNDVI == None:
#             self.get_logger().info('Funciona NDVI')
#             self.funcionaNDVI=1
 

#     def listener_callback(self, data):
#         cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
#         depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(cv_image, alpha=0.03), cv2.COLORMAP_JET)
        
#         # Calcular la profundidad en el centro de la imagen
#         self.center_depth = cv_image[cv_image.shape[0]//2, cv_image.shape[1]//2] 
#         center_depth =self.center_depth
#         #self.get_logger().info('Profundidad en el centro: "%s" milímetros' % center_depth)
        
#         # Marcar el centro de la imagen
#         marked_image = cv2.circle(depth_colormap, (self.x_calibracion, self.y_calibracion), 5, (0,0,255), -1)
        
#           # Marcar el punto de calibración
#         #marked_image = cv2.circle(marked_image, self.punto_calibra, 5, (0,255,255), -1)
        
#         #Calibración y dibujar linea

#         depth_array = np.array(cv_image, dtype=np.float32)
#         zero_depth_points = np.where(depth_array == 0) # Encuentra los puntos donde la profundidad es igual a 0
#         line_length = int(1280*40/89.41)
#         # Si hay puntos con profundidad 0, coloca la línea en la primera posición encontrada
#         if zero_depth_points[0].size > 0:
#             first_zero_point = (zero_depth_points[1][0], zero_depth_points[0][0])
#             start_point = (first_zero_point[0] - line_length // 2, first_zero_point[1])
#             end_point = (first_zero_point[0] + line_length // 2, first_zero_point[1])
#         else:
#             start_point = (cv_image.shape[1]//2 - line_length//2, cv_image.shape[0]//2)
#             end_point = (cv_image.shape[1]//2 + line_length//2, cv_image.shape[0]//2)

#         marked_image = cv2.line(marked_image, start_point, end_point, (0,0,255), 2)
        
#         # Dibujar la palabra "NDVI" encima de la línea
#         text_position = (start_point[0], start_point[1] - 10)
#         if self.NDVI_value is not None:
#             text_NDVI = 'Distance:'+ str(center_depth) + 'mm, NDVI: ' + str(self.NDVI_value)[25:32]
#         else: 
#             text_NDVI = 'NDVI: no information'
#         cv2.putText(marked_image, text_NDVI, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
        
#         # Mostrar la imagen
#         cv2.imshow('RealSense Depth Image', marked_image)
#         cv2.waitKey(1)



#     def color_listener_callback(self, data):
#         cv_image2 = self.bridge_color.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
#         # Marcar el centro de la imagen
#         marked_image2 = cv2.circle(cv_image2, (self.x_calibracion, self.y_calibracion), 5, (0,0,255), -1)
        
#         # Dibujar la línea horizontal
#         line_length = int(1280*40/89.41)   #int(0.72 * center_depth) Falta calibrar
#         start_point = (self.x_calibracion - line_length//2, self.y_calibracion)
#         end_point = (self.x_calibracion + line_length//2, self.y_calibracion)
#         marked_image2 = cv2.line(marked_image2, start_point, end_point, (0,0,255), 2)
#         # Dibujar la palabra "NDVI" encima de la línea
#         text_position = (start_point[0], start_point[1] - 10)
#         if self.NDVI_value is not None:
#             text_NDVI = 'Distance:'+ str(self.center_depth) + 'mm, NDVI: ' + str(self.NDVI_value)[25:32]
#         else: 
#             text_NDVI = 'NDVI: no information'
#         cv2.putText(marked_image2, text_NDVI, text_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2)
#         # Añadir la palabra 'Hola' en la parte inferior izquierda de la imagen
#         donde_posicion = (10, marked_image2.shape[0] - 10)  # (x, y) - 10 px de margen
#         donde=self.donde
#         cv2.putText(marked_image2, donde, donde_posicion, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    


#         # Mostrar la imagen
#         cv2.imshow('RealSense Color Image', marked_image2)
#         cv2.waitKey(1)

#     def save_ndvi_value(self):
#         if self.NDVI_value is not None:
#             ndvi_value = float(str(self.NDVI_value)[26:32])
            
#              # Leer el archivo CSV actual, agregar el nuevo valor y guardarlo
#             try:
#                 df = pd.read_csv(self.csv_file)
#             except FileNotFoundError:
#                 # Si el archivo no existe, se crea uno nuevo con la columna 'NDVI_value'
#                 df = pd.DataFrame(columns=['NDVI_value'])
        
#             df = df._append({'NDVI_value': ndvi_value}, ignore_index=True)
#             df.to_csv(self.csv_file, index=False)
            
#             #self.get_logger().info(f'NDVI value {ndvi_value} saved to CSV.')
# # def save_ndvi_value(self):
# #     if self.NDVI_value is not None:
# #         ndvi_value = float(self.NDVI_value.data)
            
# #         # Leer el archivo CSV actual, agregar el nuevo valor y guardarlo
# #         df = pd.read_csv(self.csv_file)
# #         df = df.append({'NDVI_value': ndvi_value}, ignore_index=True)
# #         df.to_csv(self.csv_file, index=False)
            
# #         self.get_logger().info(f'NDVI value {ndvi_value} saved to CSV.')

# def main(args=None):
#     rclpy.init(args=args)
#     real_sense_depth_node = RealSenseDepthNode()
#     rclpy.spin(real_sense_depth_node)
#     real_sense_depth_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()