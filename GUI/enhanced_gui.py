# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtGui import QPixmap
from PyQt5.QtCore import QTimer, QDateTime, QProcess

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(903, 948)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setStyleSheet('background-color: #f1faee;')
        
        # Adjustments to the layout, centering logo and title
        self.main_layout = QtWidgets.QVBoxLayout(self.centralwidget)
        
        # Logo and title centered at the top
        self.intro_widget = QtWidgets.QWidget(self.centralwidget)
        self.intro_layout = QtWidgets.QHBoxLayout(self.intro_widget)
        
        self.logo_label = QtWidgets.QLabel(self.intro_widget)
        self.logo_label.setPixmap(QPixmap("Logo_HUET.png"))
        self.logo_label.setFixedSize(170, 170)
        self.logo_label.setScaledContents(True)
        
        self.intro_text = QtWidgets.QLabel(self.intro_widget)

        self.intro_text.setText(
            "<h1 style='text-align: center; color: #007FFF;'>Đại học Công Nghệ - Đại học Quốc gia Hà Nội</h1>"
            "<h2 style='text-align: center; color: #264653;'>Tên: Nguyễn Văn Sơn</h2>"
            "<h2 style='text-align: center; color: #393D3F;'><b>GVHD:</b> Hoàng Văn Mạnh</h2>"
            "<p style='text-align: center; font-size: 17px; color: #4059AD;'><b>Đề tài:</b> NGHIÊN CỨU THUẬT TOÁN ĐIỀU HƯỚNG ROBOT NÔNG NGHIỆP CÓ ỨNG DỤNG TRÍ TUỆ NHÂN TẠO</p>"
        )
        self.intro_text.setAlignment(QtCore.Qt.AlignCenter)
        self.intro_text.setStyleSheet("background-color: #e9c46a; padding: 10px; border-radius: 10px;")
        
        # Center layout addition
        self.intro_layout.addWidget(self.logo_label)
        self.intro_layout.addWidget(self.intro_text)
        self.main_layout.addWidget(self.intro_widget)
        
        # Thêm phần hiển thị ngày giờ
        self.datetime_label = QtWidgets.QLabel(self.centralwidget)
        self.datetime_label.setAlignment(QtCore.Qt.AlignRight)
        self.main_layout.addWidget(self.datetime_label)
        
        # Thiết lập Timer để cập nhật ngày giờ
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_datetime)
        self.timer.start(1000)  # Cập nhật mỗi giây

        # Tạo layout chính giữa Control Panel, Video và Terminal
        self.content_layout = QtWidgets.QHBoxLayout()

        # Tạo phần điều khiển bên trái
        self.control_panel = QtWidgets.QWidget(self.centralwidget)
        

        self.control_panel.setFixedWidth(400)

        self.control_panel.setObjectName("control_panel")

        self.control_panel.setStyleSheet("background-color: #97D8C4; padding: 2px; border-radius: 10px;")
        
        # Layout cho phần điều khiển
        self.control_layout = QtWidgets.QVBoxLayout(self.control_panel)
        
        # Thêm các thanh trượt (sliders) vào phần điều khiển
        self.slider_label = QtWidgets.QLabel("Control Panel", self.control_panel)
        self.slider_label.setAlignment(QtCore.Qt.AlignCenter)
        self.control_layout.addWidget(self.slider_label)
        
        self.slider_x = QtWidgets.QSlider(QtCore.Qt.Horizontal, self.control_panel)
        self.slider_x.setRange(185, 470)
        self.slider_x.setValue(185)
        self.slider_x_label = QtWidgets.QLabel("Nozzle height:", self.control_panel)
        self.control_layout.addWidget(self.slider_x_label)
        self.control_layout.addWidget(self.slider_x)
                    
        self.slider_y = QtWidgets.QSlider(QtCore.Qt.Horizontal, self.control_panel)
        self.slider_y.setRange(50, 234)
        self.slider_y.setValue(50)
        self.slider_y_label = QtWidgets.QLabel("Left nozzle position:", self.control_panel)
        self.control_layout.addWidget(self.slider_y_label)
        self.control_layout.addWidget(self.slider_y)
        
        self.slider_z = QtWidgets.QSlider(QtCore.Qt.Horizontal, self.control_panel)
        self.slider_z.setRange(50, 234)
        self.slider_z.setValue(50)
        self.slider_z_label = QtWidgets.QLabel("Right nozzle position:", self.control_panel)
        self.control_layout.addWidget(self.slider_z_label)
        self.control_layout.addWidget(self.slider_z)

        self.center_rb_L_label = QtWidgets.QLabel("Left Position: ", self.control_panel)
        self.control_layout.addWidget(self.center_rb_L_label)

        self.center_rb_R_label = QtWidgets.QLabel("Right Position: ", self.control_panel)
        self.control_layout.addWidget(self.center_rb_R_label)
 
        # Thêm phần nhập và hiển thị giá trị Z World
        self.z_world_layout = QtWidgets.QHBoxLayout()  # Tạo layout ngang

        self.z_world_label_title = QtWidgets.QLabel("Z World Value:", self.control_panel)
        self.z_world_input = QtWidgets.QLineEdit(self.control_panel)
        self.z_world_input.setPlaceholderText("Enter Z World value...")

        # Thêm QLabel và QLineEdit vào layout ngang
        self.z_world_layout.addWidget(self.z_world_label_title)
        self.z_world_layout.addWidget(self.z_world_input)

        # Thêm layout ngang này vào layout chính của control_panel
        self.control_layout.addLayout(self.z_world_layout)
        
        # Thêm phần chế độ Tự động và Thủ công
        self.control_layout.addWidget(QtWidgets.QLabel("Mode:", self.control_panel))
        self.radio_auto = QtWidgets.QRadioButton("Auto", self.control_panel)
        self.radio_manual = QtWidgets.QRadioButton("Manual", self.control_panel)
        self.radio_manual.setChecked(True)  # Đặt chế độ mặc định là Thủ công
        self.control_layout.addWidget(self.radio_auto)
        self.control_layout.addWidget(self.radio_manual)
        
        # Thêm các nút vào phần điều khiển
        self.control_layout.addStretch(1)
        self.oke_button = QtWidgets.QPushButton("Oke", self.control_panel)  # Nút OK mới thay cho Start
        self.start_button = QtWidgets.QPushButton("Start", self.control_panel)
        self.stop_button = QtWidgets.QPushButton("Stop", self.control_panel)
        self.control_layout.addWidget(self.oke_button)
        self.control_layout.addWidget(self.start_button)
        self.control_layout.addWidget(self.stop_button)

        # Thêm phần điều khiển vào layout chính giữa
        self.content_layout.addWidget(self.control_panel)

        # Tạo phần hiển thị hình ảnh/video bên phải Control Panel
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setFixedSize(640, 480)
        self.label.setFrameShape(QtWidgets.QFrame.Box)
        self.label.setText("")
        self.label.setScaledContents(True)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        
        # Thêm phần hiển thị hình ảnh/video vào layout chính giữa
        self.content_layout.addWidget(self.label)

        # Thêm layout chính giữa vào layout chính
        self.main_layout.addLayout(self.content_layout)

        # Thông tin FPS và thời gian xử lý khung hình
        self.info_layout = QtWidgets.QHBoxLayout()
        self.fps_label = QtWidgets.QLabel("FPS: 0")
        self.frame_time_label = QtWidgets.QLabel("Frame time: 0 ms")
        self.angle_label = QtWidgets.QLabel("Angle: 0°")
        self.distance_label = QtWidgets.QLabel("Distance: 0 mm")
        self.wheel_speed_label = QtWidgets.QLabel("Wheel Speed:")
        self.speed_label = QtWidgets.QLabel("Speed:")
        self.info_layout.addWidget(self.fps_label)
        self.info_layout.addWidget(self.frame_time_label)
        self.info_layout.addWidget(self.angle_label)
        self.info_layout.addWidget(self.distance_label)
        self.info_layout.addWidget(self.wheel_speed_label)
        self.info_layout.addWidget(self.speed_label)
        self.main_layout.addLayout(self.info_layout)
        
        
        # Tạo phần terminal dưới phần hiển thị hình ảnh
        self.terminal = QtWidgets.QTextEdit(self.centralwidget)
        self.terminal.setMinimumSize(QtCore.QSize(0, 150))
        self.terminal.setReadOnly(True)
        self.main_layout.addWidget(self.terminal)
        
        # Thêm terminal vào layout chính
        self.main_layout.addWidget(self.terminal)
        
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1600, 22))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle("🚜 Robot Navigation GUI")

    def update_datetime(self):
        current_datetime = QDateTime.currentDateTime()
        self.datetime_label.setText(current_datetime.toString("dd-MM-yyyy hh:mm:ss"))


    def on_ready_read_output(self):
        """Đọc dữ liệu từ đầu ra và thêm vào terminal."""
        output = self.process.readAllStandardOutput().data().decode()
        self.terminal.append(output)

    def append_terminal(self, text):
        """Hàm hỗ trợ thêm văn bản vào terminal."""
        self.terminal.append(text)

    def update_fps(self, fps, frame_time_ms, distance, l_value, r_value):
        """Cập nhật nhãn FPS và thời gian xử lý khung hình."""
        self.fps_label.setText(f"FPS: {fps:.2f}")
        self.frame_time_label.setText(f"Frame time: {frame_time_ms:.2f} ms")
        self.distance_label.setText(f"Distance: {distance:.2f} m")
        self.wheel_speed_label.setText(f"L: {l_value:.2f} km/h, R: {r_value:.2f} km/h")
        self.speed_label.setText(f"Speed: {(l_value+r_value)/2:.2f} km/h")


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
