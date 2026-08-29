import socket
import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel

# main.cpp is the TCP server, this GUI is the client. Keep in sync with
# kGuiStatusPort in main.cpp.
MAIN_PROCESS_HOST = "127.0.0.1"
MAIN_PROCESS_PORT = 8765
MAIN_PROCESS_CONNECT_TIMEOUT_SEC = 1.0


def try_connect_to_main():
    """One-time check at startup: is the main control process up and
    listening? Returns a connected socket, or None if nothing answered
    within the timeout."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(MAIN_PROCESS_CONNECT_TIMEOUT_SEC)
    try:
        sock.connect((MAIN_PROCESS_HOST, MAIN_PROCESS_PORT))
        return sock
    except OSError:
        sock.close()
        return None


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Arm Operator Console")
        self.setGeometry(100, 100, 400, 300)

        print("Checking for main control process on port", MAIN_PROCESS_PORT, "...")
        self.main_process_socket = try_connect_to_main()
        self.connected_to_main = self.main_process_socket is not None
        print("Connected to main control process." if self.connected_to_main
              else "Main control process not detected — running standalone.")

        # TODO: once main.cpp is actually sending status updates over
        # gui_client_fd, receive them here (e.g. via a QTimer polling
        # self.main_process_socket.recv(...)) and update status_label.

        # Central widget holds everything else
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Layout stacks widgets vertically
        layout = QVBoxLayout()
        central_widget.setLayout(layout)

        # Example: status label
        self.status_label = QLabel("Status: Idle")
        layout.addWidget(self.status_label)

        # Example: e-stop button
        self.estop_button = QPushButton("EMERGENCY STOP")
        self.estop_button.setStyleSheet("background-color: red; color: white; font-weight: bold; font-size: 20px;")
        self.estop_button.clicked.connect(self.on_estop_clicked)
        layout.addWidget(self.estop_button)

        # Example: homing command button
        self.home_button = QPushButton("Run Homing")
        self.home_button.clicked.connect(self.on_home_clicked)
        layout.addWidget(self.home_button)

    def on_estop_clicked(self):
        self.status_label.setText("Status: E-STOP TRIGGERED")
        print("E-STOP pressed")
        if self.connected_to_main:
            try:
                self.main_process_socket.sendall(b"ESTOP")
            except OSError:
                print("Failed to send E-STOP — connection to main process lost.")
                self.connected_to_main = False
        else:
            print("Not connected to main process — E-STOP not sent.")

    def on_home_clicked(self):
        self.status_label.setText("Status: Homing...")
        print("Homing command sent")  # replace with actual IPC send later


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())