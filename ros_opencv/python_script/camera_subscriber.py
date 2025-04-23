#!/usr/bin/env python3

#biblioteca para usar o ROS com Pyhton
import rospy  

#Para poder mandar msg na forma de imagens
from sensor_msgs.msg import Image 

from std_msgs.msg import Float64

#Pacote que consiste em converter as imagens do OPenCV para o ROS (do tipo cv::Mat). 
#Serve como uma ponte (bridge) entre o OpenCV e o ROS
from cv_bridge import CvBridge

#importando o OpenCV
import cv2 

#Criando o nome para o subscriber
subscriberNodeName= 'camera_subscriber'

#Criando o nome para o topico que vai transmitir as msg de imagens.
topicName = 'video_topic'

# Variável global para armazenar o valor de confiança
conf = 0.0

#Essa funcao e uma funcao callback que sempre e chamada toda vez que as msg chegam
def callbackFunction (msg):
	#criando um objeto bridge
	bridgeObject=CvBridge()
	rospy.loginfo("received a video message/frame")
	print(f"conf: {conf}")
	#convertendo de cv_bridge to OpenCV image format
	convertedFrameBackToCV=bridgeObject.imgmsg_to_cv2(msg)

	#Mostrando a imagem na tela
	cv2.imshow("camera", convertedFrameBackToCV)
	
	#esperando um milisegundo
	cv2.waitKey(1)

def callback_conf (msg):
	global conf
	conf = msg.data  # Atualiza o valor do valor de confiança

def listener():	
	#Inicializano o no subscriber
	#anonymous=True significa que um numero aleatorio e adicionado ao nome do no subscriber
	rospy.init_node(subscriberNodeName)#, anonymous=True)
	#Aqui nos inscrevemos, especificando o nome do topico, o tipo da msg que ira receber, e o nome da funcao callback
	rospy.Subscriber(topicName, Image, callbackFunction)
	rospy.Subscriber('conf_topic', Float64, callback_conf)

	#aqui nos "spin" o codigo, significa que vamos executar ele infinitamente, ate ser precionado ctrl+c
	rospy.spin()
	cv2.destroyAllWindows()
      
if __name__ == '__main__':
    #Chama a funcao que ira inicializar o subscriber
    try:
        listener()
    except rospy.ROSInterruptException:
        pass


