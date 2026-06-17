import os
import sys
import subprocess

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    yolov5_dir = os.path.join(script_dir, "yolov5")

    subprocess.run([
        sys.executable, "train.py",
        "--data", "dataset/data.yaml",
        "--weights", "yolov5m.pt",
        "--epochs", "100",
        "--img", "640",
        "--batch", "16",
        "--project", "../model_output",
        "--name", ".",
        "--exist-ok"
    ], cwd=yolov5_dir)