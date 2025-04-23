#!/usr/bin/env python3

from ultralytics import YOLO
import cv2 as cv
from cv_bridge import CvBridge
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

# Carregando o modelo treinado
model = YOLO("18-12-24-Model-best.pt")
#model = YOLO("yolo11n-seg.pt")  # load an official model

rospy.init_node('camera_prediction')
publisher = rospy.Publisher('video_topic', Image, queue_size=60)

pub_conf=rospy.Publisher('conf_topic', Float64, queue_size=60)


rate = rospy.Rate(30)

cap = cv.VideoCapture(0)
video_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
video_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))

print(f"Dimensões do vídeo: {video_width}x{video_height}")

# Inicializando o objeto CvBridge, que é responsável por converter imagens entre OpenCV e ROS
bridgeObject = CvBridge()

# Dicionário de mapeamento das classes, importa direto do modelo
label_dict = model.names

# Cores para diferenciar cada classe
class_colors = {
    0: (0, 255, 0),    # Umidade
    1: (0, 0, 255),    # Corrosão
    2: (255, 0, 0),    # Rachadura
}

def talker():
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame)  # Realiza a predição no frame

        for result in results:
            for box in result.boxes:
                cls = int(box.cls) # Classe do objeto detectado
                conf = float(box.conf) # Confiança da detecção, indica a probabilidade de que a detecção seja válida (0 a 1)
                if conf < 0.3:
                    continue # Ignora detecções com baixa confiança (passa para a próxima iteração)

                # Calcular a área da caixa delimitadora (normalizada)
                x_center, y_center, width, height = map(float, box.xywhn[0])
                width_total = width * video_width
                height_total = height * video_height
                area = width_total * height_total

                #x1, x2, y1, y2 = map(int, box.xyxy[0])  # Coordenadas da caixa delimitadora

                # Convertendo coordenadas normalizadas para pixels
                x1 = int((x_center * video_width) - width_total / 2)
                y1 = int((y_center * video_height) - height_total / 2)
                x2 = int((x_center * video_width) + width_total / 2)
                y2 = int((y_center * video_height) + height_total / 2)
                
                # Verifica se a classe existe no dicionário, caso contrário, usa um valor padrão
                label = label_dict.get(cls, f"Classe {cls}") 
                #Verifica se a classe está no dicionário de cores, caso contrário, usa branco
                color = class_colors.get(cls, (255, 255, 255)) 
                label_text = f"{label} ({conf:.2f})"

                # Desenha a caixa delimitadora
                cv.rectangle(frame, (x1, y1), (x2, y2), color, 2)

                # Desenha o texto com o valor da confiança
                cv.putText(frame, label_text, (x1, y1 - 5), cv.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

                # Aplicar a máscara colorida com transparência
                mask = result.masks

                # Verifica se a máscara está disponível e não é nula
                if mask is not None and mask.data is not None:
                    try:
                        # Iterar sobre as máscaras disponíveis no frame
                        for i in range(mask.data.shape[0]):
                            
                            binary_mask = mask.data[i].cpu().numpy().astype(np.uint8)  # (H, W), valores 0 ou 1

                            # Cria imagem colorida com a mesma cor da classe
                            color_mask = np.zeros_like(frame, dtype=np.uint8) # imagem preta com o mesmo tamanho do frame
                            # Preenche a imagem colorida com a cor da classe
                            color_mask[:, :] = color  
                            
                            # Aplica a máscara binária sobre a imagem colorida 
                            # (Pega os pixels da máscara somenete onde binary_mask é 1)
                            mask_applied = cv.bitwise_and(color_mask, color_mask, mask=binary_mask) 

                            # Aplicar com transparência (blending)
                            # cv.addWeighted - mescla duas imagens com pesos diferentes
                            # np.where - substitui os pixels da imagem original onde a máscara é 1
                            frame = np.where(mask_applied > 0, cv.addWeighted(frame, 0.7, mask_applied, 0.3, 0), frame)

                    except Exception as e:
                        rospy.logwarn(f"Erro ao aplicar máscara: {e}")


                print(f"[{label_text}] x_center: {x_center:.4f}, y_center: {y_center:.4f}, área: {area:.2f}")
                pub_conf.publish(conf)
                rospy.loginfo('conf e publicado')
        
        # Converte o frame anotado para mensagem ROS e publica
        imageToTransmit = bridgeObject.cv2_to_imgmsg(frame)
        publisher.publish(imageToTransmit)
       
        rospy.loginfo('Frame capturado e publicado')
        
        rate.sleep()
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
