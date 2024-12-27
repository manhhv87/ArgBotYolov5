# gui_handle.py
import sys
import numpy as np
import math
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer
from code_handle import LiveStream, send_command_to_arduino, convert_cv_qt
from enhanced_gui import Ui_MainWindow
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QMouseEvent


class MainWindow(QMainWindow):
    update_terminal_signal = pyqtSignal(str)
    update_wheel_speed_signal = pyqtSignal(str, str)  # Thêm tín hiệu cho L và R
    def __init__(self):
        super().__init__()
        self.uic = Ui_MainWindow()
        self.uic.setupUi(self)

        # Thiết lập chế độ và giá trị ban đầu
        self.save_to_left = False  # Biến trạng thái, bắt đầu lưu vào center_rb_L
        self.save_to_right = False  # Biến trạng thái, bắt đầu lưu vào center_rb_L
        self.mode = 'manual'
        self.A1 = 0
        self.B1 = 0
        self.C1 = 0
        self.D1 = 0
        self.E1 = 0
        self.L = 0
        self.R = 0
        self.prev_L = 0  # Giá trị L trước đó
        self.prev_R = 0  # Giá trị R trước đó
        self.delta_L = 0
        self.delta_R = 0
        self.center_rb_L = 0
        self.center_rb_R = 0
        self.triggered_E1_update = False  # Biến để kiểm tra khi có tín hiệu "K1"
        self.wheel_speed_text = None
        self.center_rb_mat_L = None
        self.center_rb_mat_R = None
        self.distance = 0
        self.count_angle = 0
        self.sum_angle = 0
        self.average_angle = 0
        self.count_time = 0
        self.angle_time = 1000
        self.D1_time = 1000
        
        self.Zconst = 61.2
        
        self.cameraMatrix = np.array([
            [797.4981, 0, 323.1862],
            [0, 797.8648, 263.6120],
            [0, 0, 1]
        ], dtype=np.float64)

        self.distCoeffs = None

        self.rvec = np.array([
            [ 2.215027358148228  ],
            [ 0.01428595002737916],
            [-0.03574176312047295]
        ])
        
        self.tvec = np.array([
            [ -2.614781637432051],
            [142.2897382562742  ],
            [681.1037771565396  ]   
        ])
        
        self.rotationMatrix = np.array([
            [ 0.9995167393173203,     0.02321823794795779,   -0.020668847356921188 ],
            [-0.0025749034496375597, -0.6007845550434591,    -0.7994068352806711   ],
            [-0.030978342381359592,   0.7990737336740741,    -0.6004344347682586   ]
        ])
        self.timer = QTimer()

        # Connect update_terminal_signal to the terminal in UI
        self.update_terminal_signal.connect(self.uic.append_terminal)
        self.update_terminal_signal.connect(self.check_for_E1_update)
        self.update_wheel_speed_signal.connect(self.update_wheel_speed_display)
        
        # Khởi tạo LiveStream
        self.live_stream = LiveStream(index=1)
        self.live_stream.signal.connect(self.show_wedcam)
        self.live_stream.target_center_signal.connect(self.update_filtered_target_center)
        self.live_stream.update_fps_signal.connect(self.update_fps_info)

        # Chọn chế độ khi nhấn các nút radio
        self.uic.radio_auto.toggled.connect(self.set_auto_mode)
        self.uic.radio_manual.toggled.connect(self.set_manual_mode)

        # Cập nhật giá trị từ các thanh trượt
        self.uic.slider_x.valueChanged.connect(self.update_values)
        self.uic.slider_y.valueChanged.connect(self.update_values)
        self.uic.slider_z.valueChanged.connect(self.update_values)

        # Xử lý nút Start và Stop
        self.uic.oke_button.clicked.connect(self.start_command_sequence)
        self.uic.start_button.clicked.connect(self.start_capture_video)
        self.uic.stop_button.clicked.connect(self.stop_command_sequence)
        
        self.uic.z_world_input.returnPressed.connect(self.update_z_world)
        
        self.uic.label.mousePressEvent = self.get_mouse_position
        
    def update_z_world(self):
        self.Zconst =  float(self.uic.z_world_input.text())
        
    def get_mouse_position(self, event: QMouseEvent):
        if self.mode == 'auto':  # Chỉ kích hoạt khi chế độ tự động được chọn
            x = event.pos().x()
            y = event.pos().y()
            world_point = self.calculate_3d_point((x, y), self.Zconst)
            if world_point[0][0] <= 0:
                self.center_rb_mat_L= world_point
                self.uic.append_terminal(f"Vi tri voi phun trai: ({self.center_rb_mat_L[0][0]:.2f}, {self.center_rb_mat_L[1][0]:.2f}, {self.center_rb_mat_L[2][0]:.2f})")
                self.save_to_left = True
            else:
                self.center_rb_mat_R = world_point
                self.uic.append_terminal(f"Vi tri voi phun phai: ({self.center_rb_mat_R[0][0]:.2f}, {self.center_rb_mat_R[1][0]:.2f}, {self.center_rb_mat_R[2][0]:.2f})")
                self.save_to_right = True
            
            if (self.save_to_left and self.save_to_right):
                self.update_nozzle_positions(self.center_rb_mat_L,self.center_rb_mat_R)
                self.save_to_left = False
                self.save_to_right = False

    def update_wheel_speed_display(self, l_value, r_value):
        # Cập nhật GUI với giá trị tốc độ từ Arduino
        self.L = l_value
        self.R = r_value
        
    # Tính tọa độ 3D từ 2D
    def calculate_3d_point(self, uvPoint, Z):
        uvPoint = np.array([[uvPoint[0]], [uvPoint[1]], [1]], dtype=np.float64)
        leftSideMat = np.linalg.inv(self.rotationMatrix) @ np.linalg.inv(self.cameraMatrix) @ uvPoint
        rightSideMat = np.linalg.inv(self.rotationMatrix) @ self.tvec
        s = (Z + rightSideMat[2, 0]) / leftSideMat[2, 0]
        P = np.linalg.inv(self.rotationMatrix) @ (s * (np.linalg.inv(self.cameraMatrix) @ uvPoint) - self.tvec)
        return P
    
    # Cập nhật vị trí vòi phun    
    def update_nozzle_positions(self,center_rb_mat_L,center_rb_mat_R):
        self.uic.center_rb_L_label.setText(f"Left Position: {round(-center_rb_mat_L[0][0],2)}")
        self.uic.center_rb_R_label.setText(f"Right Position: {round(+center_rb_mat_R[0][0],2)}")
        angle = self.calculate_angle((center_rb_mat_L[0][0],center_rb_mat_L[1][0]),(center_rb_mat_R[0][0],center_rb_mat_R[1][0]))
        if 0 <= (234 + center_rb_mat_L[0][0]) <= 184 and 0 <= (234 - center_rb_mat_R[0][0]) <= 184:
            self.center_rb_L = int(234 + center_rb_mat_L[0][0])
            self.center_rb_R = int(234 - center_rb_mat_R[0][0])
            self.uic.append_terminal(f"L{self.center_rb_L} R{self.center_rb_R}")
        else:
            self.uic.append_terminal("Out of range.")
        
    def calculate_angle(self, A, B):
        """
        Tính góc giữa đường thẳng đi qua A và B với trục OX.
        
        Parameters:
            A (tuple): Điểm A (x1, y1)
            B (tuple): Điểm B (x2, y2)
        
        Returns:
            float: Góc tính bằng độ.
        """
        x1, y1 = A
        x2, y2 = B
        
        # Tính toán góc bằng arctan2
        angle_radians = math.atan2(y2 - y1, x2 - x1)
        
        # Chuyển đổi từ radian sang độ
        angle_degrees = math.degrees(angle_radians)
        
        # Đảm bảo góc trong khoảng [0, 360)
        if angle_degrees < 0:
            angle_degrees += 360
        
        return angle_degrees

    def set_auto_mode(self):
        self.mode = 'auto'

    def set_manual_mode(self):
        self.mode = 'manual'
        
    def update_fps_info(self, fps, frame_time_ms):
        self.uic.update_fps(fps, frame_time_ms, self.distance, self.delta_L, self.delta_R)

    def update_values(self):
        self.uic.slider_x_label.setText(f"Nozzle height: {self.uic.slider_x.value()}")
        self.uic.slider_y_label.setText(f"Left nozzle position: {self.uic.slider_y.value()}")
        self.uic.slider_z_label.setText(f"Right nozzle position: {self.uic.slider_z.value()}")

    # Tính toán tọa độ X1 được viết trong báo cáo
    def calculate_x_from_y(self, y, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        a = (y2 - y1) / (x2 - x1)
        b = y1 - a * x1
        x = (y - b) / a
        return x
    
    # Cập nhật giá trị E1 và D1
    def update_filtered_target_center(self, value):
        if value is None:
            self.E1 = 0  # Chỉ đặt E1 về 0 khi không có bounding box, giữ nguyên D1
            self.uic.angle_label.setText(f"Angle: °")
            self.angle_time = 1000
            self.D1_time = 1000
        else:
            mid_position = self.calculate_3d_point((value[0], value[1]), self.Zconst)
            self.D1 = value[0] - int(self.calculate_x_from_y(value[1],(318, 379),(331, 46))) # Cập nhật D1 khi có target_center
            self.D1_time = value[0] - int(self.calculate_x_from_y(value[1],(318, 379),(331, 46))) # Cập nhật D1 khi có target_center
            # Tính góc theo radian
            angle_radian = math.atan2(mid_position[0][0], mid_position[1][0])

            # Chuyển góc từ radian sang độ
            angle_degree = math.degrees(angle_radian)
            self.angle_time = angle_degree
            self.uic.angle_label.setText(f"Angle: {angle_degree:.2f}°")
            self.E1 = value[2] # cập nhật giá trị E1 là số lượng bounding box
            
            if self.triggered_E1_update:
                self.count_angle = self.count_angle + 1
                self.sum_angle = self.sum_angle + angle_degree
                self.average_angle = self.sum_angle/self.count_angle
            print(f"aver {self.average_angle:.2f}, count {self.count_angle:.2f}")

    def check_for_E1_update(self, text):
        """Kiểm tra nếu có tín hiệu "Trigger E1 Update" để cập nhật E1."""
        if "Done" in text: # Khi vòi phun đến vị trí đặt
            self.triggered_E1_update = True


    def start_command_sequence(self):
        # Khởi tạo giá trị ban đầu
        if self.mode == 'manual':
            self.A1 = 470 - self.uic.slider_x.value()
            self.C1 = 234 - self.uic.slider_y.value()
            self.B1 = 234 - self.uic.slider_z.value()
            self.D1 = 0
            self.E1 = 0  # Đặt E1 ban đầu là 0
        else:
            self.A1 = 470 - self.uic.slider_x.value()
            self.C1 = self.center_rb_L
            self.B1 = self.center_rb_R
            self.D1 = 0
            self.E1 = 0

        # Gửi lệnh đầu tiên
        send_command_to_arduino(self.A1, self.B1, self.C1, self.D1, self.E1, self.update_terminal_signal, self.update_wheel_speed_signal)

        # Bắt đầu gửi lệnh đều đặn
        self.timer.timeout.connect(self.send_continuous_command)
        self.timer.start(400)  # Gửi lệnh mỗi 0.4 giây

    def send_continuous_command(self):
        
        if self.triggered_E1_update:
            send_command_to_arduino(self.A1, self.B1, self.C1, 0, 1, self.update_terminal_signal, self.update_wheel_speed_signal)
        else:
            send_command_to_arduino(self.A1, self.B1, self.C1, 0, 1, self.update_terminal_signal, self.update_wheel_speed_signal)

        self.delta_L = (float(self.L) - float(self.prev_L)) * 3.141592653589793 * 145 * 0.0036 / 800 / 0.4
        self.delta_R = (float(self.R) - float(self.prev_R)) * 3.141592653589793 * 145 * 0.0036 / 800 / 0.4
        self.distance = (float(self.L) + float(self.R)) * 3.141592653589793 * 145 / 800 / 2

        self.prev_L = float(self.L)
        self.prev_R = float(self.R)
        
        # Lưu giá trị vào file txt
        if self.triggered_E1_update:
            self.count_time = self.count_time + 0.4
            with open("ket_qua_3.txt", "a") as file:
                file.write(f"{self.count_time:.2f}\t{self.angle_time:.2f}\t{self.D1_time:.2f}\n")

    def stop_command_sequence(self):
        # Dừng gửi lệnh và tắt chương trình
        self.timer.stop()
        self.live_stream.stop()
        QApplication.quit()

    def start_capture_video(self):
        """Bắt đầu hiển thị luồng video nếu chưa chạy."""
        if not self.live_stream.is_running:  # Kiểm tra nếu luồng chưa chạy
            self.live_stream.start()
            self.uic.append_terminal("Video stream started.")

    def show_wedcam(self, cv_img):
        qt_img = convert_cv_qt(cv_img)
        self.uic.label.setPixmap(qt_img)

    def closeEvent(self, event):
        # Đảm bảo dừng luồng video khi đóng cửa sổ
        self.live_stream.stop()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_win = MainWindow()
    main_win.show()
    sys.exit(app.exec())
