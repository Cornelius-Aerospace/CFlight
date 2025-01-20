import sys
import signal
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame, QSpacerItem, QSizePolicy, QGridLayout, QPushButton
)
from PyQt5.QtGui import QColor, QPixmap, QFont, QFontDatabase
from PyQt5.QtCore import Qt, QTimer

INDICATOR_STYLE = "font-size: 16px; font-style: italic; color: lightgrey; padding-left: 5px; padding-right: 5px;"
STRONG_INDICATOR_STYLE = "font-size: 20px; font-style: italic; color: white; padding-left: 5px; padding-right: 5px;"
class Dashboard(QWidget):
    def __init__(self):
        super().__init__()
        QFontDatabase.addApplicationFont('font/SpaceGrotesk-Regular.ttf')
        QFontDatabase.addApplicationFont('font/SpaceGrotesk-Bold.ttf')
        self.setWindowTitle("CFlight Ground Station")
        self.setStyleSheet("background-color: #111418; color: white; font-family: 'Space Grotesk';")
        self.setGeometry(100, 100, 1200, 800)
        
        
        # Main layout
        main_layout = QVBoxLayout()
        top_bar_frame = QFrame()
        # Add a grey outline to the top bar
        # Assign an object name to the top_bar_frame (prevents children from also getting a border)
        top_bar_frame.setObjectName("topBarFrame")
        # Use the object name in the stylesheet to apply the border specifically to top_bar_frame
        top_bar_frame.setStyleSheet("#topBarFrame { border: 2px solid #293038; border-radius: 10px;}")
        # Navigation bar section
        top_bar = QVBoxLayout()
        inner_top_bar = QHBoxLayout()
        title_label = QLabel("CFlight Ground Station")
        title_label.setAlignment(Qt.AlignLeft)
        title_label.setFont(QFont("Space Grotesk", 24, QFont.Bold))
        title_label.setStyleSheet("color: white; padding: 1px;")

        time_label = QLabel("T+00:00")
        time_label.setAlignment(Qt.AlignCenter)
        time_label.setFont(QFont("Space Grotesk", 24, QFont.Bold))
        time_label.setStyleSheet("color: white; padding: 5; border: 1px solid white; border-radius: 5px;")
        
        # Top Bar Buttons
        self.flight_action_button = QPushButton("Calibrate")
        self.flight_action_button.setStyleSheet("background-color: blue; color: white; font-size: 16px; font-weight: bold; padding: 13px; border-radius: 5px;")
        self.flight_action_button.clicked.connect(self.calibrate)
        
        # Add the title label to the top bar
        inner_top_bar.addWidget(title_label)
        inner_top_bar.addStretch(1)
        inner_top_bar.addWidget(self.flight_action_button)
        inner_top_bar.addStretch(8)
        inner_top_bar.addWidget(time_label)
        top_bar.addLayout(inner_top_bar)
        top_bar_frame.setLayout(top_bar)
        top_bar_frame.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Maximum)
        main_layout.addWidget(top_bar_frame)

        # System status row layout
        # Status (data value) for the status
        self.gscs_connection_status = QLabel("Disconnected")
        self.gscs_connection_status.setStyleSheet("color: red; font-size: 14px; font-weight: bold;")
        self.gscs_connection_status.setAlignment(Qt.AlignCenter)
        
        self.lora_connection_status = QLabel("Disconnected")
        self.lora_connection_status.setStyleSheet("color: red; font-size: 14px; font-weight: bold;")
        self.lora_connection_status.setAlignment(Qt.AlignCenter)
        
        self.espnow_connection_status = QLabel("Disconnected")
        self.espnow_connection_status.setStyleSheet("color: red; font-size: 14px; font-weight: bold;")
        self.espnow_connection_status.setAlignment(Qt.AlignCenter)
        
        self.rocket_status = QLabel("Disconnected")
        self.rocket_status.setStyleSheet("color: red; font-size: 14px; font-weight: bold;")
        self.rocket_status.setAlignment(Qt.AlignCenter)
        
        self.comm_link_latency = QLabel("Offline")
        self.comm_link_latency.setStyleSheet("color: red; font-size: 14px; font-weight: bold;")
        self.comm_link_latency.setAlignment(Qt.AlignCenter)
        
        
        status_row = QHBoxLayout()
        status_row.addWidget(self.create_status_widget("GSCS Connection", self.gscs_connection_status))
        status_row.addWidget(self.create_status_widget("LoRa Connection", self.lora_connection_status))
        status_row.addWidget(self.create_status_widget("EspNow Connection", self.espnow_connection_status))
        status_row.addWidget(self.create_status_widget("Rocket Status", self.rocket_status))
        status_row.addWidget(self.create_status_widget("Comm Link Latency", self.comm_link_latency))

        top_bar.addLayout(status_row)
        
        # Telemetry layout
        telemetry_row = QHBoxLayout()
    
        # Altitude telemetry box
        self.altitude_box = self.create_telemetry_box("Altitude")
        
        self.baro_altitude_label = QLabel("no data")
        self.baro_altitude_label.setStyleSheet("font-size: 24px; color: white;")
        self.gps_altitude_label = QLabel("no data")
        self.gps_altitude_label.setStyleSheet("font-size: 24px; color: white;")
        # Baro alt
        baro_altitude_layout = QHBoxLayout()
        baro_altitude_layout.addWidget(self.baro_altitude_label)
        baro_indicator = QLabel("(Barometer)")
        baro_indicator.setStyleSheet(INDICATOR_STYLE)
        baro_altitude_layout.addWidget(baro_indicator)
        baro_altitude_layout.setAlignment(Qt.AlignCenter)
        self.altitude_box.layout().addLayout(baro_altitude_layout)
        # GPS alt
        gps_altitude_layout = QHBoxLayout()
        gps_altitude_layout.addWidget(self.gps_altitude_label)
        gps_indicator = QLabel("(GPS)")
        gps_indicator.setStyleSheet(INDICATOR_STYLE)
        gps_altitude_layout.addWidget(gps_indicator)
        gps_altitude_layout.setAlignment(Qt.AlignCenter)
        self.altitude_box.layout().addLayout(gps_altitude_layout)

        self.altitude_box.layout().addStretch(1)
        telemetry_row.addWidget(self.altitude_box)

        # Velocity telemetry box
        self.velocity_box = self.create_telemetry_box("Velocity")

        self.derived_velocity_label = QLabel("no data")
        self.gps_velocity_label = QLabel("no data")
        self.derived_velocity_label.setStyleSheet("font-size: 24px; color: white;")
        self.gps_velocity_label.setStyleSheet("font-size: 24px; color: white;")
        # Derived velocity
        derived_velocity_layout = QHBoxLayout()
        derived_velocity_layout.addWidget(self.derived_velocity_label)
        velocity_indicator = QLabel("(computed)")
        velocity_indicator.setStyleSheet(INDICATOR_STYLE)
        derived_velocity_layout.addWidget(velocity_indicator)
        derived_velocity_layout.setAlignment(Qt.AlignCenter)
        self.velocity_box.layout().addLayout(derived_velocity_layout)
        # GPS velocity
        gps_velocity_layout = QHBoxLayout()
        gps_velocity_layout.addWidget(self.gps_velocity_label)
        gps_velocity_indicator = QLabel("(GPS)")
        gps_velocity_indicator.setStyleSheet(INDICATOR_STYLE)
        gps_velocity_layout.addWidget(gps_velocity_indicator)
        gps_velocity_layout.setAlignment(Qt.AlignCenter)
        self.velocity_box.layout().addLayout(gps_velocity_layout)

        self.velocity_box.layout().addStretch(1)
        telemetry_row.addWidget(self.velocity_box)

        # Acceleration telemetry box
        self.acceleration_box = self.create_telemetry_box("Acceleration")

        self.x_acceleration_label = QLabel("no data")
        self.y_acceleration_label = QLabel("no data")
        self.z_acceleration_label = QLabel("no data")
        self.x_acceleration_label.setStyleSheet("font-size: 24px; color: white;")
        self.y_acceleration_label.setStyleSheet("font-size: 24px; color: white;")
        self.z_acceleration_label.setStyleSheet("font-size: 24px; color: white;")
        # Indicators
        x_indicator = QLabel("(X)")
        x_indicator.setStyleSheet(STRONG_INDICATOR_STYLE)
        y_indicator = QLabel("(Y)")
        y_indicator.setStyleSheet(STRONG_INDICATOR_STYLE)
        z_indicator = QLabel("(Z)")
        z_indicator.setStyleSheet(STRONG_INDICATOR_STYLE)
        # Layout
        acceleration_layout = QGridLayout()
        acceleration_layout.addWidget(x_indicator, 0, 0)
        acceleration_layout.addWidget(self.x_acceleration_label, 0, 1)
        acceleration_layout.addWidget(y_indicator, 1, 0)
        acceleration_layout.addWidget(self.y_acceleration_label, 1, 1)
        acceleration_layout.addWidget(z_indicator, 2, 0)
        acceleration_layout.addWidget(self.z_acceleration_label, 2, 1)

        self.acceleration_box.layout().addLayout(acceleration_layout)
        telemetry_row.addWidget(self.acceleration_box)
        


        # Additional Telemetry Data (row 2)
        telemetry_row2 = QHBoxLayout()
        
        self.temperature_label = QLabel("25°C")
        self.temperature_label.setStyleSheet("font-size: 24px; color: white;")
        self.battery_label = QLabel("85%")
        self.battery_label.setStyleSheet("font-size: 24px; color: white;")
        self.signal_strength_label = QLabel("-75 dBm")
        self.signal_strength_label.setStyleSheet("font-size: 24px; color: white;")
        # GPS Coordinates
        self.gps_lat_label = QLabel("0.52124")
        self.gps_lat_label.setStyleSheet("font-size: 24px; color: white;")
        self.gps_long_label = QLabel("-0.283756")
        self.gps_long_label.setStyleSheet("font-size: 24px; color: white;")

        telemetry_row2.addWidget(self.create_telemetry_box("Temperature"))
        telemetry_row2.addWidget(self.create_telemetry_box("Battery Level"))
        telemetry_row2.addWidget(self.create_telemetry_box("Signal Strength"))
        telemetry_row2.addWidget(self.create_telemetry_box("GPS Coordinates"))


        # Add a holder for the rows
        telemetry_layout = QVBoxLayout()
        telemetry_layout.addLayout(telemetry_row)
        telemetry_layout.addLayout(telemetry_row2)

        # Frame to hold flight info
        flight_info_frame = self.create_flight_info()

        # Horizontal layout to hold telemetry layouts and flight panel frame
        bottom_layout = QGridLayout()
        bottom_layout.addWidget(flight_info_frame, 0, 0, 1, 1)
        bottom_layout.addLayout(telemetry_layout, 0, 1, 1, 4)

        # Frame to hold GPS map
        gps_map_frame = QFrame()
        gps_map_frame.setObjectName("gpsMapFrame")
        gps_map_frame.setStyleSheet("#gpsMapFrame { border: 2px solid #293038; }")
        gps_map_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Frame to hold the graph
        graph_frame = QFrame()
        graph_frame.setObjectName("graphFrame")
        graph_frame.setStyleSheet("#graphFrame { border: 2px solid #293038; }")
        graph_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Frame to hold the orientation visuals
        orientation_visuals_frame = QFrame()
        orientation_visuals_frame.setObjectName("orientationVisualsFrame")
        orientation_visuals_frame.setStyleSheet("#orientationVisualsFrame { border: 2px solid #293038; }")
        orientation_visuals_frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)


        # Horizontal layout to hold graphs, maps, visuals, etc.
        visuals_layout = QHBoxLayout()
        visuals_layout.addWidget(gps_map_frame)
        visuals_layout.addWidget(graph_frame)
        visuals_layout.addWidget(orientation_visuals_frame)

        # Vertical layout to hold the bottom layout and visuals layout
        dashboard_layout = QVBoxLayout()
        dashboard_layout.addLayout(visuals_layout)
        dashboard_layout.addLayout(bottom_layout)

        main_layout.addLayout(dashboard_layout)
        self.setLayout(main_layout)

        # Spacer to increase the gap between SSR and telemetry layouts
        main_layout.addSpacerItem(QSpacerItem(0, 20, QSizePolicy.Minimum, QSizePolicy.Fixed))

        self.loopIteration = 0
        self.loopTimer = QTimer(self)
        self.loopTimer.setSingleShot(False)
        self.loopTimer.setInterval(10) 
        self.loopTimer.timeout.connect(self.mainLoop)
        self.loopTimer.start()

        # Set layout and show widget
        self.setLayout(main_layout)

    def mainLoop(self):
        self.loopIteration += 1
        if self.loopIteration % 100 == 0:
            self.updateStatusValue(self.gscs_connection_status, "Connected", "green")
            self.updateStatusValue(self.lora_connection_status, "Connected", "green")
            self.updateStatusValue(self.espnow_connection_status, "Connected", "green")
            self.updateStatusValue(self.rocket_status, "Connected", "green")
            self.updateStatusValue(self.comm_link_latency, "10ms", "green")

            self.updateTelemetryValue(self.baro_altitude_label, "1000m") 
        elif self.loopIteration % 50 == 0:
            self.updateStatusValue(self.gscs_connection_status, "Connecting", "yellow")
            self.updateStatusValue(self.lora_connection_status, "Connecting", "yellow")
            self.updateStatusValue(self.espnow_connection_status, "Connecting", "yellow")
            self.updateStatusValue(self.rocket_status, "Connecting", "yellow")
            self.updateStatusValue(self.comm_link_latency, "50ms", "yellow")

            self.updateTelemetryValue(self.baro_altitude_label, "500m")

    def updateStatusValue(self, statusValue, newValue, color):
        statusValue.setText(newValue)
        statusValue.setStyleSheet(f"color: {color}; font-size: 14px; font-weight: bold;")
        statusValue.setAlignment(Qt.AlignCenter)
    
    def updateTelemetryValue(self, telemetryValue, newValue, color="white"):
        telemetryValue.setText(newValue)
        telemetryValue.setStyleSheet(f"font-size: 24px; color: {color};")
        telemetryValue.setAlignment(Qt.AlignCenter)

    def calibrated(self):
        self.flight_action_button.setEnabled(False)
        self.flight_action_button.setText("ARM")
        self.flight_action_button.setStyleSheet("background-color: red; color: white; font-size: 16px; font-weight: bold; padding: 10px; border-radius: 5px;")
        self.flight_action_button.clicked.connect(self.arm)
        self.flight_action_button.clicked.disconnect(self.calibrate)
        self.flight_action_button.setEnabled(True)

    def armed(self):
        self.flight_action_button.setEnabled(False)
        self.flight_action_button.setText("DISARM")
        self.flight_action_button.setStyleSheet("background-color: green; color: white; font-size: 16px; font-weight: bold; padding: 10px; border-radius: 5px;")
        self.flight_action_button.clicked.connect(self.disarm)
        self.flight_action_button.clicked.disconnect(self.arm)
        self.flight_action_button.setEnabled(True)
    
    def disarmed(self):
        self.flight_action_button.setEnabled(False)
        self.flight_action_button.setText("CALIBRATE")
        self.flight_action_button.setStyleSheet("background-color: blue; color: white; font-size: 16px; font-weight: bold; padding: 10px; border-radius: 5px;")
        self.flight_action_button.clicked.connect(self.calibrate)
        self.flight_action_button.clicked.disconnect(self.disarm)
        self.flight_action_button.setEnabled(True)

    def arm(self):
        # TODO: Implement arm functionality
        print("Arming the rocket...")
        self.armed()

    def disarm(self):
        # TODO: Implement disarm functionality
        print("Disarming the rocket...")
        self.disarmed()
    
    def calibrate(self):
        # TODO: Implement calibrate functionality
        print("Starting calibration...")
        self.calibrated()

    def create_flight_info(self):
        master_frame = QFrame()
        master_frame.setObjectName("flightPanelFrame")
        master_frame.setStyleSheet("#flightPanelFrame { border: 2px solid #293038; }")
        outer_layout = QVBoxLayout()
        master_frame.setLayout(outer_layout)
        # Add a label 
        flight_info_label = QLabel("Flight Information")
        flight_info_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #bbbbbb;")
        flight_info_label.setAlignment(Qt.AlignCenter)
        outer_layout.addWidget(flight_info_label)
        inner_frame = QFrame()
        outer_layout.addWidget(inner_frame)
        inner_frame.setStyleSheet("background-color: #293038; border-radius: 10px; padding: 0px;")
        inner_frame.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Expanding)
        inner_layout = QGridLayout()
        inner_frame.setLayout(inner_layout)
        

        
        return master_frame

    def create_status_widget(self, label_text, value_lable):
        """Creates a status indicator widget with a label and status text separated by a line."""
        widget = QWidget()
        widget.setStyleSheet("""
            background-color: #293038;
            border-radius: 10px;
            padding: 0px;
        """)
        
        layout = QVBoxLayout()
        
        # Label (title) for the status at the top with padding
        label = QLabel(label_text)
        label.setStyleSheet("font-weight: bold; font-size: 14px; color: #bbbbbb; padding: 0px;")
        label.setAlignment(Qt.AlignCenter)
        # Also center align the label
        

        # Grey separator line under the title
        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setFrameShadow(QFrame.Sunken)
        separator.setStyleSheet("color: #888888; background-color: #888888; border: 0px; padding: 0px; margin: 0px;")
        
        layout.addWidget(label)
        layout.addWidget(separator)
        layout.addWidget(value_lable)
        widget.setLayout(layout)
        # Shrinks the widget to the minimum size
        widget.setSizePolicy(QSizePolicy.Minimum, QSizePolicy.Maximum)
        #widget.setMaximumHeight(90)
        
        return widget

    def create_telemetry_box(self, label_text):
        """Creates a modern, curved-edge box for displaying telemetry data."""
        widget = QWidget()
        widget.setStyleSheet("""
            background-color: #293038;
            border-radius: 10px;
            padding: 0px;
            color: white;
        """)
        
        layout = QVBoxLayout()
        
        label = QLabel(label_text)
        label.setStyleSheet("font-size: 18px; font-weight: bold; color: #bbbbbb;")
        label.setAlignment(Qt.AlignCenter | Qt.AlignTop)
        layout.addWidget(label)
        widget.setLayout(layout)
        widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        return widget
    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            print("Escape key pressed, exiting...")
            self.close()
        
            

if __name__ == "__main__":
    app = QApplication(sys.argv)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    dashboard = Dashboard()

    dashboard.show()
    # Start the application event loop
    sys.exit(app.exec_())
