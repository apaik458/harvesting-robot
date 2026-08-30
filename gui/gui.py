import socket
import sys
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QHBoxLayout, QWidget, QLabel

STATUS_POLL_INTERVAL_MS = 200

# main.cpp is the TCP server, this GUI is the client
MAIN_PROCESS_HOST = "127.0.0.1"
MAIN_PROCESS_PORT = 8765
MAIN_PROCESS_CONNECT_TIMEOUT_SEC = 1.0

FAULT_OPTIONS = [
    ("Servo Disconnected", "SERVO_DISCONNECTED"),
    ("Excessive Torque", "EXCESSIVE_TORQUE"),
    ("Camera Disconnected", "CAMERA_DISCONNECTED"),
]

# One-time check at startup to establish a socket connection with main process
def try_connect_to_main():
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
        self.setGeometry(100, 100, 700, 400)

        print("Checking for main control process on port", MAIN_PROCESS_PORT, "...")
        self.main_process_socket = try_connect_to_main()
        self.connected_to_main = self.main_process_socket is not None
        print("Connected to main control process." if self.connected_to_main
              else "Main control process not detected — running standalone.")

        self.active_fault = None
        self.fault_buttons = {}
        self._recv_buffer = ""

        if self.connected_to_main:
            self.main_process_socket.setblocking(False)
            self.status_timer = QTimer(self)
            self.status_timer.timeout.connect(self._poll_status)
            self.status_timer.start(STATUS_POLL_INTERVAL_MS)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        outer_layout = QVBoxLayout()
        central_widget.setLayout(outer_layout)

        self.status_label = QLabel("Status: Idle")
        outer_layout.addWidget(self.status_label)

        columns_layout = QHBoxLayout()
        outer_layout.addLayout(columns_layout)

        # Left column of buttons for fault injection
        fault_column = QVBoxLayout()
        fault_column.addWidget(QLabel("Fault Injection"))
        for label, code in FAULT_OPTIONS:
            button = QPushButton(f"Inject {label}")
            button.clicked.connect(
                lambda _checked, c=code, l=label: self.on_fault_button_clicked(c, l))
            fault_column.addWidget(button)
            self.fault_buttons[code] = button
        fault_column.addStretch()
        columns_layout.addLayout(fault_column)

        # Centre column for emergency stop button
        center_column = QVBoxLayout()
        center_column.addStretch()
        center_row = QHBoxLayout()
        center_row.addStretch()

        self.estop_button = QPushButton("EMERGENCY\nSTOP")
        self.estop_button.setFixedSize(160, 160)
        self.estop_button.setStyleSheet(
            "QPushButton {"
            "  background-color: red;"
            "  color: white;"
            "  font-weight: bold;"
            "  font-size: 16px;"
            "  border-radius: 80px;"
            "  border: 4px solid darkred;"
            "}"
            "QPushButton:pressed { background-color: darkred; }"
        )
        self.estop_button.clicked.connect(self.on_estop_clicked)
        center_row.addWidget(self.estop_button)

        center_row.addStretch()
        center_column.addLayout(center_row)
        center_column.addStretch()
        columns_layout.addLayout(center_column, 1)

        # Right column: telemetry at the top, quit pinned to the bottom
        right_column = QVBoxLayout()
        self.telemetry_label = QLabel(
            "Motor 1: --\nMotor 2: --\nMotor 3: --\nCamera:  --")
        self.telemetry_label.setStyleSheet("font-family: monospace;")
        right_column.addWidget(self.telemetry_label)
        right_column.addStretch()

        self.quit_button = QPushButton("Quit")
        self.quit_button.setStyleSheet("font-weight: bold;")
        self.quit_button.clicked.connect(self.on_quit_clicked)
        right_column.addWidget(self.quit_button)
        columns_layout.addLayout(right_column)

    def _send_command(self, message):
        if not self.connected_to_main:
            print(f"Not connected to main process — '{message}' not sent.")
            return
        try:
            self.main_process_socket.sendall(message.encode())
        except OSError:
            print(f"Failed to send '{message}' — connection to main process lost.")
            self.connected_to_main = False

    def _poll_status(self):
        """Called periodically by status_timer to read STATUS updates main.cpp
        sends each control loop cycle (motor connections/torques, camera
        connection). Non-blocking, same style as main.cpp's own polling of
        GUI commands."""
        if not self.connected_to_main:
            return
        try:
            data = self.main_process_socket.recv(4096)
        except BlockingIOError:
            return
        except OSError:
            print("Lost connection to main process while polling status.")
            self.connected_to_main = False
            return

        if not data:
            print("Main process closed the connection.")
            self.connected_to_main = False
            return

        self._recv_buffer += data.decode(errors="ignore")
        while "\n" in self._recv_buffer:
            line, self._recv_buffer = self._recv_buffer.split("\n", 1)
            self._handle_status_line(line)

    def _handle_status_line(self, line):
        if not line.startswith("STATUS:"):
            return
        try:
            fields = dict(pair.split("=") for pair in line[len("STATUS:"):].split(","))

            def status(key):
                return "OK" if fields[key] == "1" else "DISCONNECTED"

            self.telemetry_label.setText(
                f"Motor 1: {float(fields['t1']):.2f} Nm [{status('m1')}]\n"
                f"Motor 2: {float(fields['t2']):.2f} Nm [{status('m2')}]\n"
                f"Motor 3: {float(fields['t3']):.2f} Nm [{status('m3')}]\n"
                f"Camera:  [{status('cam')}]"
            )
        except (KeyError, ValueError):
            print(f"Malformed status line from main process: {line!r}")

    def on_estop_clicked(self):
        self.status_label.setText("Status: E-STOP TRIGGERED")
        print("E-STOP pressed")
        self._send_command("ESTOP")

    def on_quit_clicked(self):
        print("Quit pressed — shutting down main process and GUI")
        self._send_command("QUIT")
        QApplication.instance().quit()

    def on_fault_button_clicked(self, code, label):
        button = self.fault_buttons[code]
        if self.active_fault == code:
            # Same button pressed again: clear the fault
            self._send_command("CLEAR")
            button.setText(f"Inject {label}")
            self.active_fault = None
            self.status_label.setText("Status: Idle")
        else:
            # Switching faults (or injecting the first one): reset whichever button was previously armed
            if self.active_fault is not None:
                prev_label = next(l for l, c in FAULT_OPTIONS if c == self.active_fault)
                self.fault_buttons[self.active_fault].setText(f"Inject {prev_label}")
            self._send_command(f"INJECT:{code}")
            button.setText(f"Clear {label}")
            self.active_fault = code
            self.status_label.setText(f"Status: Injecting {label}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
