#!/usr/bin/env python3

#biblioteca para usar o ROS com Pyhton
import rospy  
import numpy as np

#Limitamos o intervalo da cor desejada (nesse caso o amarelo)
lower = np.array([0, 120, 70])
upper = np.array([180, 255, 255])

#Cor desejada, utilizada como referencia
color_hsv = np.array([180, 200, 100])

#Para poder mandar msg na forma de imagens
from sensor_msgs.msg import Image 
from std_msgs.msg import Float64

#Pacote que consiste em converter as imagens do OPenCV para o ROS (do tipo cv::Mat). 
#Serve como uma ponte (bridge) entre o OpenCV e o ROS
from cv_bridge import CvBridge
 
#importando o OpenCV
import cv2

#Criando o nome para o publisher
publisherNodeName= 'camera_publisher'

#Criando o nome para o topico que vai transmitir as msg de imagens.
topicName = 'video_topic'


#inicializando o no
rospy.init_node(publisherNodeName)#, anonymous=True)

#Criando um Criando um objeto publisher, especificando o nome do topico, o tipo de msg (Image)
#e definindo o tamanho do buffer (queue_size)
publisher=rospy.Publisher(topicName, Image, queue_size=60)
pub_distance_color=rospy.Publisher('distance_topic', Float64, queue_size=60)

#Taxa de transmissao das mensagens (30hz)
rate = rospy.Rate(30)

#Criando o objeto que vai capturar o video
videoCaptureObject=cv2.VideoCapture(0)

#Criando o objeto CvBridge que vai ser usado para converter as imagens OPenCV para mensagens de imagens ROS
bridgeObject = CvBridge()

def talker():
	#O loop infinito captura as imagens, converte em msg ROS e transmite pelo topico
	while not rospy.is_shutdown():
		#retorna dois valores, o primeiro indicando sucesso/falha na operacao, e o segundo o frame capturado
		returnValue, capturedFrame = videoCaptureObject.read()
		image = cv2.cvtColor(capturedFrame, cv2.COLOR_BGR2HSV) #convertendo o color-space da imagem capturada, nesse caso de RGB para HSV
		mask = cv2.inRange(image, lower, upper) #extraindo a cor limitada pelos intervalos anteriores

		# Encontrar pixels da cor de referencia
		color_pixels = image[mask > 0]  # Pega apenas pixels dessa cor 

		if len(color_pixels) > 0:
        	# Calcular a média das cores HSV dos pixels detectados
			avg_color = np.mean(color_pixels, axis=0)

        	# Calcular a distância euclidiana entre a média detectada e a cor desejada
			distance = np.linalg.norm(avg_color - color_hsv)
		else:
			distance = None  # Nenhum pixel da cor detectado
		
		#Criando o contorno envolta da cor delimitada
		contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
		if len(contours) != 0:
			for contour in contours:
				if cv2.contourArea(contour) > 500:
					x,y,w,h = cv2.boundingRect(contour)
					capturedFrame = cv2.rectangle(capturedFrame, (x,y), (x+w, y+h), (0, 0, 255), 3)

		#Se o frame de video foi capturado, publicar no topico
		if returnValue == True:
			rospy.loginfo('Video frame captured and published')
			print("distance color: ", distance)
			#Convertendo msg OpenCV para ROS	
			imageToTransmit = bridgeObject.cv2_to_imgmsg(capturedFrame)
			#Publicando a imagem convertida atraves do topico
			publisher.publish(imageToTransmit)
			pub_distance_color.publish(distance)
		rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
