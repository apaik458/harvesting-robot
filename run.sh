# Runs the main robot control process alongside the GUI. This timing guarantees that the
# socket connection between processes will be established

./build/arm_control &
sleep 0.5
python3 gui/gui.py
