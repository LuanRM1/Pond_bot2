import cv2

def verificar_webcam():
    # Inicializa a captura de vídeo (0 é geralmente o índice para a webcam padrão)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        return "Erro: Não foi possível acessar a webcam."

    # Tenta ler um frame da webcam
    ret, frame = cap.read()

    # Libera a captura de vídeo
    cap.release()

    if ret:
        return frame
    else:
        return "Erro: Não foi possível ler a imagem da webcam."

# Chama a função e imprime o resultado
print(verificar_webcam())

