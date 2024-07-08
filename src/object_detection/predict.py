from ultralytics import YOLO
import time

start = time.time()
model = YOLO("best100ep.pt")
result = model.predict("Video.mov", save=True, save_txt=True)[0]
stop = time.time()
print("Total time:", round(stop-start,2))
'''
for box in result.boxes:
  class_id = result.names[box.cls[0].item()]
  cords = box.xyxy[0].tolist()
  cords = [round(x) for x in cords]
  conf = round(box.conf[0].item(), 2)
  print("Object type:", class_id)
  print("Coordinates:", cords)
  print("Probability:", conf)
  print("---")
'''