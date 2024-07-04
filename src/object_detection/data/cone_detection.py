from ultralytics import YOLO
import os


data_yaml_path = '/home/jblevins32/ML_Work/cone_dataset/data.yaml'

# Ensure the path is correct
print(f"Using data.yaml located at: {data_yaml_path}")
print(f"Does the file exist? {os.path.isfile(data_yaml_path)}")

# Load a pre-trained YOLOv8 model
cone_model = YOLO('yolov8n.pt')

# Train the model
cone_model.train(data='data.yaml', epochs=1, imgsz=640, batch=16)

# Perform inference on the test image
# results = cone_model.predict()

# # Print the detected objects and their confidence scores
# print(results.pandas().xywh[0])