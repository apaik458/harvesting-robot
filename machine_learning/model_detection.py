from ultralytics import YOLO
import cv2
import os

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model = YOLO(os.path.join(script_dir, "model_output/train/weights/best.pt"))

    cap = cv2.VideoCapture(4) # "4" is the camera's index, may need to change for different camera

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame, verbose=False, conf=0.5)  # filters at model level

        stems = []
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            class_name = results[0].names[class_id]
            confidence = box.conf[0].item()

            if class_name == "stem":
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                stems.append((cx, cy))
                print(f"Stem at ({cx:.0f}, {cy:.0f}), confidence: {confidence:.2f}")

        # now plot() only draws what passed the 0.6 threshold
        annotated = results[0].plot()
        cv2.imshow("Stem Detection", annotated)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()