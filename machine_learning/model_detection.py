import torch
import cv2
import os

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    model = torch.hub.load(
        os.path.join(script_dir, "yolov5"),
        'custom',
        path=os.path.join(script_dir, "model_output/weights/best.pt"),
        source='local'
    )
    model.conf = 0.3

    cap = cv2.VideoCapture(4)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame)

        stems = []
        for *box, conf, cls in results.xyxy[0].tolist():
            class_name = model.names[int(cls)]
            if class_name == "stem":
                x1, y1, x2, y2 = box
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2
                stems.append((cx, cy))
                print(f"Stem at ({cx:.0f}, {cy:.0f}), confidence: {conf:.2f}")

        annotated = results.render()[0]
        cv2.imshow("Stem Detection", annotated)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()