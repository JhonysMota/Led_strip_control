# TESTE: PASSAR OS FRAMES PELO MODELO TREINADO
# MODELO: YOLO11N

from ultralytics import YOLO
import cv2 as cv

# Parte necessária para realizão a predição
# Load a model
model = YOLO("yolo11n-seg.pt")  # load an official model
model = YOLO("results/18-12-24-Model-best.pt")  # load a custom model

# Carregar o vídeo
video_path = 'videos/tunel-02.mp4'
cap = cv.VideoCapture(video_path) # abra a câmera do drone

# DIMENSÃO DO VÍDEO : 480X848

# Obter as dimensões do vídeo
video_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))  # Largura dos frames
video_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))  # Altura dos frames

print(f"Dimensões do vídeo: {video_width}x{video_height}")

# Obter a taxa de frames por segundo (FPS) do vídeo
fps = cap.get(cv.CAP_PROP_FPS)
frame_interval = int(fps * 2)  # Intervalo de 2 segundos

frame_number = 0

# Abrir ou criar o arquivo de texto para salvar as informações
output_file = 'results/video-predictions.txt'

with open(output_file, 'a') as file:  # Abrir no modo append para adicionar sem sobrescrever
    # Loop para ler cada frame do vídeo
    while cap.isOpened():
        ret, frame = cap.read()  # 'ret' será False quando o vídeo terminar
        if not ret:
            break

        # Apenas salvar ou processar o frame a cada intervalo definido
        if frame_number % frame_interval == 0:
            results = model(frame)  # YOLO realiza a predição diretamente no frame

            for result in results:
                # Iterar sobre as bounding boxes detectadas
                for box in result.boxes:
                  # Extrair as informações desejadas
                  cls = int(box.cls)  # Classe do objeto
                  x_center, y_center, width, height = map(float, box.xywhn[0])
                  # Calculando a área
                  width_total = width * video_width             # o valor normalizado multiplicado pela largura do frame
                  height_total = height * video_height          # o valor normalizado multiplicado pela altura do frame
                  area = width_total * height_total             # valor da área = largura x altura

                  # Exibir os valores ajustados
                  print(f"x_center: {x_center:.4f}, y_center: {y_center:.4f}, width: {width:.4f}, height: {height:.4f}, area: {area:.4f}")

                  # Escrever no arquivo com o formato desejado
                  file.write(f"{frame_number} {cls} {x_center} {y_center} {width} {height} {area}\n")

            # Salvar o frame com as anotações
            frame_filename = f'results/frame_{frame_number:04d}.jpg'
            result.save(f'results/frame_{frame_number:04d}_anotated.jpg')
            print(f'Frame {frame_number} saved as {frame_filename}')

        frame_number += 1

# Liberar o vídeo
cap.release()
print("Video processing complete.")