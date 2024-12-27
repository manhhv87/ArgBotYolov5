import serial
import time
import cv2
import numpy as np
import torch
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5 import QtGui, QtCore
from PyQt5.QtGui import QPixmap
from models.common import DetectMultiBackend
from utils.general import non_max_suppression, scale_boxes
from utils.torch_utils import select_device
from utils.dataloaders import LoadStreams

# Kết nối Serial đến Arduino
ser = serial.Serial("/dev/ttyUSB0", 115200)
# ser = serial.Serial("/dev/rfcomm0", 115200)
time.sleep(2)

# Hàm gửi lệnh xuống Arduino
def send_command_to_arduino(A1, B1, C1, D1, E1, terminal_signal=None, wheel_speed_signal=None):
    command = f'G10 X{A1} Y{B1} Z{C1} A{D1} E{E1}\n'
    ser.write(command.encode('utf-8'))
    if terminal_signal:
        terminal_signal.emit(f"Sent to Arduino: {command}")

    response = ser.readline().decode('utf-8').strip()
    if terminal_signal:
        terminal_signal.emit(f"Received from Arduino: {response}")

    if "G20" in response:
        try:
            l_value = response.split(" L")[1].split(" R")[0]
            r_value = response.split(" R")[1]
            if wheel_speed_signal:
                wheel_speed_signal.emit(l_value, r_value)
        except IndexError:
            if terminal_signal:
                terminal_signal.emit("Error parsing wheel speed data from Arduino.")
    if "K1" in response and terminal_signal:
        terminal_signal.emit("Done")

# Hàm chuyển đổi OpenCV ảnh sang QPixmap
def convert_cv_qt(cv_img):
    rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    h, w, ch = rgb_image.shape
    bytes_per_line = ch * w
    convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
    return QPixmap.fromImage(convert_to_Qt_format)

class LiveStream(QThread):
    signal = pyqtSignal(np.ndarray)
    target_center_signal = pyqtSignal(object)
    update_fps_signal = pyqtSignal(float, float)

    def __init__(self, index):
        super().__init__()
        self.index = index
        self.is_running = False
        self.device = select_device('0')
        self.model = self.load_model()
        self.target_centers = []
        self.MAX_VALUES = 20
        self.last_time = time.time()
        self.fps = 0
        
        # Initialize nozzle positions
        self.left_nozzle_pos = (0, 379)
        self.right_nozzle_pos = (320, 379)
        
        self.Zconst = 61.2
        
        self.cameraMatrix = np.array([
            [797.4981, 0, 323.1862],
            [0, 797.8648, 263.6120],
            [0, 0, 1]
        ], dtype=np.float64)

        self.distCoeffs = np.array([0.0202, 0.2425, 0.0053, -3.4086e-04, 0], dtype=np.float64)

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
        
        self.axis = np.array([
            (-250, 0, 0),      # Gốc tọa độ (dịch -250 theo Ox)
            (-150, 0, 0),      # Điểm trên trục X
            (-250, 200, 0),    # Điểm trên trục Y
            (-250, 0, 50)      # Điểm trên trục Z (hướng đúng)
        ], dtype=np.float32)

        # Thêm hệ tọa độ mới với Z = 61.2
        self.axis_new_plane = np.array([
            (0, 0, 61.2),         # Gốc tọa độ mới
            (200, 0, 61.2),       # Trục X1
            (0, 500, 61.2),       # Trục Y1
            (0, 0, 100)         # Trục Z1
        ], dtype=np.float32)

    # Thay file .pt theo mong muốn
    def load_model(self):
        print(f"Sử dụng thiết bị: {self.device}")
        model = DetectMultiBackend('best.pt', device=self.device)
        model.warmup()
        return model

    def run(self):
        self.is_running = True
        # Sử dụng LoadStreams để lấy dữ liệu từ camera, thay "2" thành chỉ số của camera
        dataset = LoadStreams("2", img_size=640, stride=self.model.stride, auto=self.model.pt)

        while self.is_running:
            start_time = time.time()  # Bắt đầu đo thời gian cho một khung hình

            for path, im, im0s, vid_cap, s in dataset:
                # Cập nhật thời gian hiện tại để tính thời gian trôi qua giữa hai khung hình
                current_time = time.time()
                elapsed_time = current_time - start_time  # Thời gian trôi qua cho một khung hình
                self.fps = 1.0 / elapsed_time  # Tính FPS từ thời gian xử lý khung hình
                frame_time_ms = elapsed_time * 1000
                start_time = current_time  # Cập nhật thời gian bắt đầu cho khung hình tiếp theo

                # Phát tín hiệu cập nhật FPS và thời gian
                self.update_fps_signal.emit(self.fps, frame_time_ms)

                # Chuẩn bị ảnh cho model YOLO
                im = torch.from_numpy(im).to(self.model.device)
                im = im.half() if self.model.fp16 else im.float()
                im /= 255
                if len(im.shape) == 3:
                    im = im[None]

                # Dự đoán và lấy bounding box
                pred = self.model(im)
                pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45)

                for det in pred:
                    im0 = im0s[0].copy()
                    
                    # Vẽ hệ tọa độ gốc (X, Y, Z)
                    im0 = self.draw_axes(
                        im0,
                        self.axis,
                        ["X", "Y", "Z"],
                        [(0, 0, 255), (0, 255, 0), (255, 0, 0)]
                    )

                    # Vẽ hệ tọa độ mới (X1, Y1, Z1)
                    im0 = self.draw_axes(
                        im0,
                        self.axis_new_plane,
                        ["X1", "Y1", "Z1"],
                        [(255, 0, 255), (0, 255, 255), (255, 255, 0)]
                    )

                    # Hiển thị FPS trên khung hình
                    fps_text = f"FPS: {int(self.fps)}"
                    cv2.putText(im0, fps_text, (im0.shape[1] - 150, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                    if len(det):
                        det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                        # Tạo labels và cords từ kết quả dự đoán
                        labels = det[:, -1].cpu().numpy()
                        cords = det[:, :-1].cpu().numpy()

                        # Gọi hàm plot_boxes với labels, cords và im0
                        im0, filtered_target_center = self.plot_boxes(labels, cords, im0)
                        self.target_center_signal.emit(filtered_target_center)
                    else:
                        self.target_center_signal.emit(None)
                    self.signal.emit(im0)

                if not self.is_running:
                    break
        self.is_running = False

    def plot_boxes(self, labels, cords, frame):
        max_y = -1
        target_center = None
        filtered_target_center = None
        len_box = len(cords)
        for i, row in enumerate(cords):
            x1, y1, x2, y2 = int(row[0]), int(row[1]), int(row[2]), int(row[3])
            if y2 > max_y:
                max_y = y2
                target_center = ((x1 + x2) // 2, (y1 + y2) // 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{self.class_to_label(labels[i])} {row[4]:.2f}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        if target_center:
            cv2.circle(frame, target_center, 5, (255, 0, 0), -1)
            self.target_centers.append(target_center)
            if len(self.target_centers) > self.MAX_VALUES:
                self.target_centers.pop(0)
            if len(self.target_centers) == self.MAX_VALUES:
                avg_x = sum([c[0] for c in self.target_centers]) // self.MAX_VALUES
                avg_y = sum([c[1] for c in self.target_centers]) // self.MAX_VALUES
                filtered_target_center = (avg_x, avg_y)
            else:
                filtered_target_center = target_center
            
            X1 = int(self.calculate_x_from_y(filtered_target_center[1],(318, 379),(331, 46)))
            cv2.circle(frame, (X1, filtered_target_center[1]), 5, (0, 255, 255), -1)
            cv2.putText(frame, f"X1{str((X1, filtered_target_center[1]))}", (X1-150,filtered_target_center[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2)
            cv2.circle(frame, filtered_target_center, 5, (0, 0, 255), -1)
            cv2.putText(frame, f"X{str(filtered_target_center)}", (filtered_target_center[0]+10,filtered_target_center[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2)
            
            cv2.line(frame, filtered_target_center, (320, 379), (0, 150, 255), 2)
            cv2.line(frame, filtered_target_center, (X1, filtered_target_center[1]), (150, 0, 255), 2)

        return frame, (filtered_target_center[0], filtered_target_center[1], len_box) if filtered_target_center else None
    
    
    def calculate_3d_point(self, uvPoint):
        uvPoint = np.array([[uvPoint[0]], [uvPoint[1]], [1]], dtype=np.float64)
        leftSideMat = np.linalg.inv(self.rotationMatrix) @ np.linalg.inv(self.cameraMatrix) @ uvPoint
        rightSideMat = np.linalg.inv(self.rotationMatrix) @ self.tvec
        s = (self.Zconst + rightSideMat[2, 0]) / leftSideMat[2, 0]
        P = np.linalg.inv(self.rotationMatrix) @ (s * (np.linalg.inv(self.cameraMatrix) @ uvPoint) - self.tvec)
        return P
    
    def calculate_x_from_y(self, y, point1, point2):
        # Lấy tọa độ từ hai điểm
        x1, y1 = point1
        x2, y2 = point2
        
        # Tính hệ số a và b
        a = (y2 - y1) / (x2 - x1)
        b = y1 - a * x1

        # Tính giá trị x
        x = (y - b) / a
        return x
    
    # Hàm hiển thị trục tọa độ
    def draw_axes(self, frame, axis, axis_labels, color_set):
        # Chiếu các điểm gốc và trục lên hình ảnh
        origin = axis[0:1]  # Gốc tọa độ
        imgpts, _ = cv2.projectPoints(axis, self.rvec, self.tvec, self.cameraMatrix, None)

        origin_img = tuple(imgpts[0].ravel().astype(int))

        for i, color in enumerate(color_set):
            axis_img = tuple(imgpts[i + 1].ravel().astype(int))
            cv2.arrowedLine(frame, origin_img, axis_img, color, 2, tipLength=0.1)
            cv2.putText(frame, axis_labels[i], axis_img, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        return frame
    
    def update_nozzle_positions(self, left_x, right_x):
        """Update the positions of the left and right nozzles."""
        self.left_nozzle_pos = (left_x, 379)
        self.right_nozzle_pos = (right_x, 379)

    def class_to_label(self, x):
        return self.model.names[int(x)]

    def stop(self):
        self.is_running = False
        self.quit()
        self.wait()
