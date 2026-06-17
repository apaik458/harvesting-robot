from ultralytics import YOLO

model = YOLO("model_output/train/weights/best.pt")
model.export(format="onnx")
# creates model_output/train/weights/best.onnx