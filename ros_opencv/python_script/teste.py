from ultralytics import YOLO
import cv2 as cv

# Carregar modelo treinado
model = YOLO("18-12-24-Model-best.pt")

# Obter nomes das classes
label_dict = model.names

# Cores por classe
class_colors = {
    0: (0, 255, 0),    # Umidade
    1: (0, 0, 255),    # Corrosão
    2: (255, 0, 0),    # Rachadura
    # Adicione mais conforme necessário
}

# Captura da webcam
cap = cv.VideoCapture(0)

# Verificar se a câmera abriu corretamente
if not cap.isOpened():
    print("Erro ao acessar a webcam.")
    exit()

video_width = int(cap.get(cv.CAP_PROP_FRAME_WIDTH))
video_height = int(cap.get(cv.CAP_PROP_FRAME_HEIGHT))
print(f"Webcam iniciada com resolução: {video_width}x{video_height}")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model.predict(frame, show=False, verbose=False)

    for result in results:
        for box in result.boxes:
            cls = int(box.cls)
            conf = float(box.conf)
            if conf < 0.5:
                continue

            x_center, y_center, width, height = map(float, box.xywhn[0])
            width_total = width * video_width
            height_total = height * video_height
            area = width_total * height_total

            x1 = int((x_center * video_width) - width_total / 2)
            y1 = int((y_center * video_height) - height_total / 2)
            x2 = int((x_center * video_width) + width_total / 2)
            y2 = int((y_center * video_height) + height_total / 2)

            label = label_dict.get(cls, f"Classe {cls}")
            color = class_colors.get(cls, (255, 255, 255))

            label_text = f"{label} ({conf:.2f}) - Área: {area:.1f}"

            # Caixa
            cv.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            # Texto
            cv.putText(frame, label_text, (x1, y1 - 5), cv.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            print(f"[{label_text}] x_center: {x_center:.4f}, y_center: {y_center:.4f}, área: {area:.2f}")

    # Mostrar a imagem
    cv.imshow("Webcam com Detecção", frame)

    # Pressione 'q' para sair
    if cv.waitKey(1) & 0xFF == ord('q'):
        break

# Limpar tudo
cap.release()
cv.destroyAllWindows()
print("Encerrado.")
