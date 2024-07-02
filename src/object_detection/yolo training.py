from ultralytics import YOLO

model = YOLO("yolov8n.pt")
model.train(data="data/data.yaml",epochs=100)