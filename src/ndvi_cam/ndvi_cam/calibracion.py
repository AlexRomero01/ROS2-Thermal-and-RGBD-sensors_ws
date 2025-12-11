
import rclpy
from rclpy.node import Node
from ultralytics_ros.msg import YoloResult
from std_msgs.msg import Float32MultiArray  
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class YoloDetectionSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_detection_subscriber')
        self.get_logger().info('Inicio: coloque objeto en el centro de la zona de irradiación de frente, evita que hayan otros objetos en la zona')
        # self.color_subscription = self.create_subscription(
        #     Image,
        #     '/yolo_image',
        #     self.color_listener_callback2,
        #     10)
        
        self.subscription = self.create_subscription(
            YoloResult,
            '/yolo_result',
            self.detection_callback,
            10)
        
        self.a=0
        self.posiciones=['de frente','desde arriba','desde abajo','desde la derecha','desde la izquierda']
        self.lista_puntos_x=[]
        self.lista_puntos_y=[]
        self.media_x=None
        self.media_y=None
        self.bridge_color = CvBridge() 
    
    # def color_listener_callback2(self, data):
    #     cv_image2 = self.bridge_color.imgmsg_to_cv2(data, desired_encoding='bgr8')
    #     cv2.imshow('Calibration process', cv_image2)
    #     cv2.waitKey(1)
        

    def detection_callback(self, msg):
        if self.a in [0,1,2,3,4] :
            self.lista_posicion = Float32MultiArray()  # Inicializamos el mensaje
            time.sleep(5)

            for detection in msg.detections.detections:
                if detection.results:
                    self.get_logger().info('Enfoque el objeto '+ self.posiciones[self.a])
                    result = detection.results[0]
                    class_id = result.hypothesis.class_id
                    score = result.hypothesis.score
                    center_x = detection.bbox.center.position.x
                    center_y = detection.bbox.center.position.y
                    self.lista_puntos_x.append(center_x)
                    self.lista_puntos_y.append(center_y)
                    self.get_logger().info('Posición:('+str(center_x)+', '+str(center_y)+')')
                else:
                    self.get_logger().info('No se detectaron objetos en esta imagen.')
            time.sleep(5)
            self.a=self.a+1

        elif self.a ==5:
            self.media_x=sum(self.lista_puntos_x)/len(self.lista_puntos_x)
            self.media_y=sum(self.lista_puntos_y)/len(self.lista_puntos_y)

            with open('position.txt', 'w') as archivo:
                archivo.write(str(int(self.media_x))+' '+str(int(self.media_y)))
                self.a=self.a+1
        elif self.a==6:  
            self.get_logger().info('Fin calibración')  
            self.a=self.a+1          



def main(args=None):
    rclpy.init(args=args)
    yolo_detection_subscriber = YoloDetectionSubscriber()
    rclpy.spin(yolo_detection_subscriber)
    yolo_detection_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





# for detection in msg.detections.detections:
#                 if detection.results:
#                     self.get_logger().info('Enfoque el objeto desde arriba')
#                     time.sleep(5)
#                     result = detection.results[0]
#                     class_id = result.hypothesis.class_id
#                     score = result.hypothesis.score
#                     center_x = detection.bbox.center.position.x
#                     center_y = detection.bbox.center.position.y
#                     self.lista_posicion.data.append(center_x)
#                     self.lista_posicion.data.append(center_y)
#                     self.lista_puntos_x.append(center_x)
#                     self.lista_puntos_y.append(center_y)
#                     self.get_logger().info('Posición:('+str(center_x)+', '+str(center_y)+') '+ str(class_id))
#                 else:
#                     self.get_logger().info('No se detectaron objetos en esta imagen.')
#             time.sleep(1)
# for detection in msg.detections.detections:
#                 if detection.results:
#                     self.get_logger().info('Enfoque el objeto desde abajo')
#                     time.sleep(10)
#                     result = detection.results[0]
#                     class_id = result.hypothesis.class_id
#                     score = result.hypothesis.score
#                     center_x = detection.bbox.center.position.x
#                     center_y = detection.bbox.center.position.y
#                     self.lista_posicion.data.append(center_x)
#                     self.lista_posicion.data.append(center_y)
#                     self.lista_puntos_x.append(center_x)
#                     self.lista_puntos_y.append(center_y)
#                     self.get_logger().info('Posición:('+str(center_x)+', '+str(center_y)+') '+ str(class_id))
#                 else:
#                     self.get_logger().info('No se detectaron objetos en esta imagen.')
#             for detection in msg.detections.detections:
#                 if detection.results:
#                     self.get_logger().info('Enfoque el objeto desde la derecha')
#                     time.sleep(10)
#                     result = detection.results[0]
#                     class_id = result.hypothesis.class_id
#                     score = result.hypothesis.score
#                     center_x = detection.bbox.center.position.x
#                     center_y = detection.bbox.center.position.y
#                     self.lista_posicion.data.append(center_x)
#                     self.lista_posicion.data.append(center_y)
#                     self.lista_puntos_x.append(center_x)
#                     self.lista_puntos_y.append(center_y)
#                     self.get_logger().info('Posición:('+str(center_x)+', '+str(center_y)+') '+ str(class_id))
#                 else:
#                     self.get_logger().info('No se detectaron objetos en esta imagen.')
#             for detection in msg.detections.detections:
#                 if detection.results:
#                     self.get_logger().info('Enfoque el objeto desde la izquierda')
#                     time.sleep(10)
#                     result = detection.results[0]
#                     class_id = result.hypothesis.class_id
#                     score = result.hypothesis.score
#                     center_x = detection.bbox.center.position.x
#                     center_y = detection.bbox.center.position.y
#                     self.lista_posicion.data.append(center_x)
#                     self.lista_posicion.data.append(center_y)
#                     self.lista_puntos_x.append(center_x)
#                     self.lista_puntos_y.append(center_y)
#                     self.get_logger().info('Posición:('+str(center_x)+', '+str(center_y)+') '+ str(class_id))
#                 else:
#                     self.get_logger().info('No se detectaron objetos en esta imagen.')