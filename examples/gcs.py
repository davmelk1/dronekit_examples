import sys
import time

import collections
# Monkey patch collections to support dronekit on Python 3.10+
if sys.version_info >= (3, 10):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping

from PyQt5.QtWidgets import (
    QApplication, QWidget, QPushButton, QLineEdit,
    QVBoxLayout, QLabel, QMessageBox, QGridLayout, QHBoxLayout
)
from PyQt5.QtCore import QRunnable, QThreadPool, QTimer
from PyQt5.QtCore import Qt
class Worker(QRunnable):
    def __init__(self, fn):
        super().__init__()
        self.fn = fn

    def run(self):
        self.fn()

from dronekit import VehicleMode, connect

class GCS(QWidget):
    def __init__(self, parent=None):
        super().__init__()

        self.setFocusPolicy(Qt.StrongFocus)

        self.thread_pool = QThreadPool.globalInstance()
        self.connection_timer = QTimer(self)
        self.connection_timer.timeout.connect(self.check_connection)
        self.connection_timer.start(100)

        self.control_timer = QTimer(self)
        self.control_timer.timeout.connect(self.update_rc)
        self.control_timer.start(50)  # 20 Hz

        self.pressed_keys = set()
        self.rc = {
            1: 1500,  # Roll
            2: 1500,  # Pitch
            3: 1000,  # Throttle
            4: 1500,  # Yaw
        }

        self.vehicle = None
        self.is_called = False
        self.setWindowTitle("Ground control station")
        self.resize(500, 150)
        self.connection_dst = QLineEdit("127.0.0.1:14550")
        self.connect_button = QPushButton("Connect")
        self.disconnect_button = QPushButton("Disconnect")
        self.arm_button = QPushButton("Arm")
        self.guided_button = QPushButton("Guided")
        self.stabilize_button = QPushButton("Stabilize")
        self.loiter_button = QPushButton("Loiter")
        self.land_button = QPushButton("Land")
        self.rtl_button = QPushButton("RTL")
        self.takeoff_input = QLineEdit("15")
        self.takeoff_button = QPushButton("Takeoff")


        main_layout = QGridLayout(self)
        layout1 = QHBoxLayout(self)
        layout2 = QGridLayout(self)
        layout1.addWidget(self.connect_button)
        layout1.addWidget(self.disconnect_button)
        main_layout.addLayout(layout1, 0, 0)
        main_layout.addWidget(self.connection_dst, 1, 0)
        layout2.addWidget(self.guided_button, 0, 0)
        layout2.addWidget(self.arm_button, 1, 0)
        layout2.addWidget(self.stabilize_button, 2, 0)
        layout2.addWidget(self.loiter_button, 0, 1)
        layout2.addWidget(self.land_button, 1, 1)
        layout2.addWidget(self.rtl_button, 2, 1)
        main_layout.addLayout(layout2, 2, 0)
        layout3 = QHBoxLayout(self)
        layout3.addWidget(self.takeoff_button)
        layout3.addWidget(self.takeoff_input)
        main_layout.addLayout(layout3, 3, 0)

        self.connect_button.clicked.connect(lambda: self.run_async(self.connect_drone))
        self.disconnect_button.clicked.connect(lambda: self.run_async(self.disconnect_drone))
        self.disconnect_button.setDisabled(True)
        self.guided_button.clicked.connect(lambda: self.run_async(lambda: self.change_mode("GUIDED")))
        self.stabilize_button.clicked.connect(lambda: self.run_async(lambda: self.change_mode("STABILIZE")))
        self.loiter_button.clicked.connect(lambda: self.run_async(lambda: self.change_mode("LOITER")))
        self.land_button.clicked.connect(lambda: self.run_async(lambda: self.change_mode("LAND")))
        self.rtl_button.clicked.connect(lambda: self.run_async(lambda: self.change_mode("RTL")))
        self.arm_button.clicked.connect(lambda: self.run_async(self.arm_drone))
        self.takeoff_button.clicked.connect(lambda: self.run_async(self.arm_and_takeoff))

        self.status_label = QLabel("Disconnected")
        main_layout.addWidget(self.status_label, 4, 0)

        # self.run_async(self.connect_drone)


    def run_async(self, fn):
        self.thread_pool.start(Worker(fn))

    def connect_drone(self):
        self.is_called = True
        self.connect_button.setDisabled(True)
        self.connection_dst.setDisabled(True)
        print(f"connecting to drone at {self.connection_dst.text()}")
        self.vehicle = connect(self.connection_dst.text(), wait_ready=True)
        if self.vehicle:
            print("Connected")
            self.vehicle.add_attribute_listener('armed', self.armed_callback)
        self.is_called = False


    def disconnect_drone(self):
        if self.vehicle is None:
            return
        print('disconnecting from drone')
        self.vehicle.close()
        self.vehicle = None

    def check_connection(self):
        if (self.vehicle is not None and self.vehicle.last_heartbeat < 3) or self.is_called:
            self.disconnect_button.setEnabled(True)
            self.connect_button.setDisabled(True)
            self.connection_dst.setDisabled(True)
            if self.vehicle:
                self.status_label.setText(f"Connected | mode: {self.vehicle.mode.name} | altitude: {self.vehicle.location.global_relative_frame.alt}\n"
                                      f"{self.vehicle.channels.overrides}".replace("'1'",'r').replace("'2'",'p').replace("'3'",'t').replace("'4'",'y'))
        else:
            self.connect_button.setEnabled(True)
            self.connection_dst.setEnabled(True)
            self.disconnect_button.setDisabled(True)
            self.status_label.setText("Disconnected")
            return



    def armed_callback(self, vehicle, attribute_name, value):
        if value:
            self.arm_button.setText("Disarm")
        else:
            self.arm_button.setText("Arm")

    def change_mode(self, mode):
        if self.vehicle is None:
            return
        print('changing mode to ' + mode)
        self.vehicle.mode = VehicleMode(mode)
        time.sleep(1)

    def arm_drone(self):
        if self.vehicle is None:
            return
        self.vehicle.armed = not self.vehicle.armed

    def arm_and_takeoff(self):
        if not self.vehicle.armed:
            print("Arming motors")
            self.vehicle.armed = True

            while not self.vehicle.armed:
                print(" Waiting for arming...")
                time.sleep(1)

        print("Taking off!")
        self.vehicle.simple_takeoff(self.takeoff_input.text())

        while True:
            if self.vehicle.mode.name != "GUIDED":
                break
            current_altitude = self.vehicle.location.global_relative_frame.alt
            print(" Altitude: %s" % current_altitude)
            if current_altitude >= float(self.takeoff_input.text()) * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def keyPressEvent(self, event):
        if not event.isAutoRepeat():
            self.pressed_keys.add(event.key())

    def keyReleaseEvent(self, event):
        if not event.isAutoRepeat():
            self.pressed_keys.discard(event.key())

    def update_rc(self):
        if not self.vehicle:
            return

        step = 50

        # Roll
        self.rc[1] = 1500
        if Qt.Key_Left in self.pressed_keys:
            self.rc[1] += step
        if Qt.Key_Right in self.pressed_keys:
            self.rc[1] -= step

        # Pitch
        self.rc[2] = 1500
        if Qt.Key_Up in self.pressed_keys:
            self.rc[2] += step
        if Qt.Key_Down in self.pressed_keys:
            self.rc[2] -= step

        # Throttle
        if Qt.Key_W in self.pressed_keys:
            self.rc[3] = min(2000, self.rc[3] + 30)
        elif Qt.Key_S in self.pressed_keys:
            self.rc[3] = max(1000, self.rc[3] - 30)

        # Yaw
        self.rc[4] = 1500
        if Qt.Key_A in self.pressed_keys:
            self.rc[4] -= step
        if Qt.Key_D in self.pressed_keys:
            self.rc[4] += step

        # Send RC overrides
        self.vehicle.channels.overrides = {
            str(ch): value for ch, value in self.rc.items()
        }

    def focusOutEvent(self, event):
        if self.vehicle is None:
            return
        self.pressed_keys.clear()
        self.rc[1] = 1500
        self.rc[2] = 1500
        self.rc[3] = 1000
        self.rc[4] = 1500
        self.vehicle.channels.overrides = {
            str(ch): value for ch, value in self.rc.items()
        }
        # super().focusOutEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = GCS()
    window.show()
    sys.exit(app.exec_())
