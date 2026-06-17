from ultralytics import YOLO
import os

if __name__ == "__main__":
    model = YOLO("yolov8s.pt")
    
    output = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model_output")
    
    model.train(
        data="strawberry_stem_model_yolov8/data.yaml",
        epochs=100,
        imgsz=640,
        batch=16,
        project=output,
        exist_ok=True
    )