from ultralytics import YOLO

# Load a pretrained YOLOv8n-pose Pose model
model = YOLO("yolov8n-pose.pt")

# Run inference on an image
results = model("bus.jpg")  # results list

# View results
for r in results:
    print(r.keypoints.numpy())  # print the Keypoints object containing the detected keypoints