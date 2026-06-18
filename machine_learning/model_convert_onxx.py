import os
import sys
import subprocess

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    yolov5_dir = os.path.join(script_dir, "yolov5")

    subprocess.run([
        sys.executable, "export.py",
        "--weights", "../model_output/weights/best.pt",
        "--include", "onnx",
    ], cwd=yolov5_dir)