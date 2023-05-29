#!usr/bin/env python2

# Importa las bibliotecas necesarias
import cv2
import rospy
import os
import ctypes
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from std_msgs.msg import Int32


# Carga la biblioteca compartida
lib = ctypes.CDLL('/home/betnoa/midterm_ws/src/captura_imagen/scripts/multiplica_coordenadas/coordinate_multiplier.so')
multiply_coordinates = lib.multiply_coordinates

# Crear los publicadores ROS para las coordenadas multiplicadas
coords_x_publisher = rospy.Publisher('multiplied_coordinates_x', Int32, queue_size=10)
coords_y_publisher = rospy.Publisher('multiplied_coordinates_y', Int32, queue_size=10)

def detect_green_object(image):
    # Convierte la imagen a formato HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define el rango de colores verdes en HSV
    lower_green = (36, 25, 25)
    upper_green = (86, 255, 255)

    # Crea una mascara para el rango de colores verdes
    mask = cv2.inRange(hsv_image, lower_green, upper_green)

    # Aplica la mascara a la imagen original
    result = cv2.bitwise_and(image, image, mask=mask)

    # Convierte result a una matriz NumPy valida
    result = np.array(result)

    return result, mask


def calculate_object_coordinates(contour):
    # Calcula el centroide del contorno
    moments = cv2.moments(contour)
    cx = int(moments['m10'] / (moments['m00'] + 1e-5))  # Agrega 1e-5 para evitar divisiones por cero
    cy = int(moments['m01'] / (moments['m00'] + 1e-5))

    return cx, cy



def save_image(image):
    # Obten el directorio actual
    current_dir = os.getcwd()

    # Crea un directorio "images" si no existe
    images_dir = os.path.join(current_dir, "images")
    if not os.path.exists(images_dir):
        os.makedirs(images_dir)

    # Genera un nombre unico para la imagen
    filename = os.path.join(images_dir, "green_object_image.jpg")

    # Guarda la imagen en disco
    cv2.imwrite(filename, image)
    print("Imagen guardada como:", filename)

def image_callback(image_msg):
    # Convierte el mensaje de imagen a formato OpenCV
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")

    # Detecta el objeto verde en la imagen
    result, mask = detect_green_object(image)

    # Encuentra los contornos en la mascara
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Si se encontraron contornos
    if len(contours) > 0:
        # Encuentra el contorno mas grande
        largest_contour = max(contours, key=cv2.contourArea)

        # Calcula las coordenadas del objeto verde
        cx, cy = calculate_object_coordinates(largest_contour)

        cx_ctypes = ctypes.c_int(cx)
        cy_ctypes = ctypes.c_int(cy)

        # Multiplica las coordenadas por 100 usando la biblioteca de C++
        cx_ctypes = ctypes.c_int(cx)
        cy_ctypes = ctypes.c_int(cy)
        multiply_coordinates(cx_ctypes, cy_ctypes)

        result_x = cx
        result_y = cy
        
        # Multiplicar las coordenadas por 100
        result_x_multiplied = result_x * 100
        result_y_multiplied = result_y * 100

        # Dibuja un circulo en las coordenadas del objeto
        cv2.circle(result, (cx, cy), 5, (0, 255, 0), -1)

        # Muestra las coordenadas del objeto
        print("Coordenadas del objeto verde - X:", cx, "Y:", cy)

     
        # Crear un objeto Int32 para las coordenadas multiplicadas
        result_x_multiplied = Int32()
        result_y_multiplied = Int32()

        # Asignar las coordenadas multiplicadas
        result_x_multiplied.data = result_x * 100
        result_y_multiplied.data = result_y * 100

        # Publicar las coordenadas multiplicadas en los topicos
        coords_x_publisher.publish(result_x_multiplied)
        coords_y_publisher.publish(result_y_multiplied)


    # Muestra la imagen resultante
    cv2.imshow("Green Object Detection", np.array(result))
    cv2.waitKey(1)


def capture_image():
    # Inicializa el nodo ROS
    rospy.init_node('image_capture_node')

    # Crea un objeto CvBridge
    bridge = CvBridge()

    # Crea un objeto VideoCapture para acceder a la camara
    cap = cv2.VideoCapture(0)

    # Crea un publicador ROS para enviar la imagen capturada
    image_publisher = rospy.Publisher('image_topic', Image, queue_size=10)

    rate = rospy.Rate(10)  # Frecuencia de publicacion (10 Hz)

    # Crea un suscriptor ROS para recibir la imagen capturada
    rospy.Subscriber('image_topic', Image, image_callback)

    while not rospy.is_shutdown():
        # Captura un fotograma de la camara
        ret, frame = cap.read()

        # Convierte la imagen OpenCV al formato de mensaje Image de ROS
        image_msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")

        # Publica la imagen capturada
        image_publisher.publish(image_msg)

        # Realiza la deteccion del objeto verde
        result, _ = detect_green_object(frame)

        # Muestra la imagen resultante
        cv2.imshow("Green Object Detection", result)
        cv2.waitKey(1)


        # Espera la siguiente iteracion
        rate.sleep()

    # Libera los recursos
    cap.release()

if __name__ == '__main__':
    try:

        # Crea el publicador para las coordenadas multiplicadas
        coords_publisher = rospy.Publisher('coordinates_topic', String, queue_size=10)

        # Llama a la funcion capture_image()
        capture_image()
    except rospy.ROSInterruptException:
        pass
