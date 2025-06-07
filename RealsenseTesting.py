import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import torchvision.transforms as transforms
from torchvision.models import vgg16, VGG16_Weights
from PIL import Image

# === Ustawienia modeli i klas ===
MODEL_PATH = "models/msc-VGG16.pth"
CLASS_NAMES = ['bottle', 'matches', 'mug', 'parfum', 'pen']
IMG_SIZE = 224

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# === Załadowanie modelu ===
model = vgg16(weights=VGG16_Weights.IMAGENET1K_V1)
for param in model.parameters():
    param.requires_grad = False
model.classifier[6] = torch.nn.Sequential(
    torch.nn.Dropout(0.5),
    torch.nn.Linear(model.classifier[6].in_features, len(CLASS_NAMES))
)
model.load_state_dict(torch.load(MODEL_PATH, map_location=device))
model = model.to(device)
model.eval()

# === Transformacja obrazu ===
transform = transforms.Compose([
    transforms.Resize((IMG_SIZE, IMG_SIZE)),
    transforms.ToTensor(),
    transforms.Normalize([0.5]*3, [0.5]*3)
])

# === Inicjalizacja realsense'a ===
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # konwersja na np.array (dla cv)
        color_image = np.asanyarray(color_frame.get_data())

        # przygotowanie obrazu dla modelu
        pil_img = Image.fromarray(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
        input_tensor = transform(pil_img).unsqueeze(0).to(device)

        # predykcja
        with torch.no_grad():
            output = model(input_tensor)
            _, pred = torch.max(output, 1)
            predicted_class = CLASS_NAMES[pred.item()]

        # narysowanie predykcji
        cv2.putText(color_image, f"Detected: {predicted_class}", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        # pokaz klatke
        cv2.imshow("RealSense Classification", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
