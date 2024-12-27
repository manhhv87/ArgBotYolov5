# Ultralytics YOLOv5 🚀, AGPL-3.0 license
"""
Run YOLOv5 detection inference on images, videos, directories, globs, YouTube, webcam, streams, etc.

Usage - sources:
    $ python detect.py --weights yolov5s.pt --source 0                               # webcam
                                                     img.jpg                         # image
                                                     vid.mp4                         # video
                                                     screen                          # screenshot
                                                     path/                           # directory
                                                     list.txt                        # list of images
                                                     list.streams                    # list of streams
                                                     'path/*.jpg'                    # glob
                                                     'https://youtu.be/LNwODJXcvt4'  # YouTube
                                                     'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python detect.py --weights yolov5s.pt                 # PyTorch
                                 yolov5s.torchscript        # TorchScript
                                 yolov5s.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                 yolov5s_openvino_model     # OpenVINO
                                 yolov5s.engine             # TensorRT
                                 yolov5s.mlmodel            # CoreML (macOS-only)
                                 yolov5s_saved_model        # TensorFlow SavedModel
                                 yolov5s.pb                 # TensorFlow GraphDef
                                 yolov5s.tflite             # TensorFlow Lite
                                 yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
                                 yolov5s_paddle_model       # PaddlePaddle
"""

import argparse
import csv
import os
import platform
import sys
import random
import time
import numpy as np
import math
from pathlib import Path

import torch

import tkinter as tk
from multiprocessing import Process, Value

import serial
# Kết nối với Arduino qua Serial
# sudo rfcomm bind /dev/rfcomm0 00:22:09:01:8B:00
# ser = serial.Serial("/dev/rfcomm0", 115200)  # Thay COM11 bằng cổng Serial phù hợp
ser = serial.Serial("/dev/ttyUSB0", 115200)  # Thay COM11 bằng cổng Serial phù hợp
time.sleep(2)

# Biến chia sẻ giữa các tiến trình
click_count = 0
A1 = Value('i', 0)
B1 = Value('i', 0)
C1 = Value('i', 0)
x_left = Value('i', 0)
x_right = Value('i', 0)  # Giá trị giả định cho chế độ tự động
X, Y = 0, 0
mid_x_value = Value('i', 0)
send_command = Value('b', False)  # Điều khiển việc bắt đầu gửi lệnh
box_detect = Value('b', False)  # Điều khiển việc bắt đầu gửi lệnh
mode = "auto"  # Chế độ mặc định là thủ công

mid_x_values = []  
MAX_VALUES = 20  # Số lượng giá trị cần lưu để tính trung bình

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from ultralytics.utils.plotting import Annotator, colors, save_one_box

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (
    LOGGER,
    Profile,
    check_file,
    check_img_size,
    check_imshow,
    check_requirements,
    colorstr,
    cv2,
    increment_path,
    non_max_suppression,
    print_args,
    scale_boxes,
    strip_optimizer,
    xyxy2xywh,
)
from utils.torch_utils import select_device, smart_inference_mode, time_sync


def onMouse(event, xcoord, ycoord, flags, userdata):
    global X, Y, click_count, mode

    if event == cv2.EVENT_LBUTTONDOWN:
        # Cập nhật tọa độ X, Y khi nhấn chuột
        X = xcoord
        Y = ycoord

        # Tính toán giá trị X1 từ tọa độ X, Y
        X1 = 234.44892093805064961344886777954 - 0.018276709390253895662059342799432*Y - 0.70473078530423828054999694590185*X

        # Xác định giá trị B1 và C1 theo thứ tự nhấn chuột
        
        if 20 < X1 < 193:
            x_right.value = int(193 - X1)  # Lần nhấn chuột thứ hai gán giá trị cho C1
            # print(f"Giá trị C1 được gán là: {x_right.value}")
            print(f"Giá trị C1 được gán là: {X1}")
        elif -193 < X1 < -20:
            x_left.value = int(193 + X1)  # Lần nhấn chuột đầu tiên gán giá trị cho B1
            # print(f"Giá trị B1 được gán là: {x_left.value}")
            print(f"Giá trị B1 được gán là: {X1}")
        else:
            print("Ngoài phạm vi")
        
        # if click_count == 0:
        #     x_left.value = int(X1)  # Lần nhấn chuột đầu tiên gán giá trị cho B1
        #     print(f"Giá trị B1 được gán là: {x_left.value}")
        # elif click_count == 1:
        #     x_right.value = int(X1)  # Lần nhấn chuột thứ hai gán giá trị cho C1
        #     print(f"Giá trị C1 được gán là: {x_right.value}")

        # Cập nhật B1 và C1 nếu đang ở chế độ tự động
        print(f"che do: {mode}")
        if mode == "auto":
            B1.value = x_left.value
            C1.value = x_right.value

        # Tăng click_count để biết lần nhấn chuột tiếp theo
        # click_count += 1
        # if click_count == 2:
        #     click_count = 0

            
        
def update_values():
    # Cập nhật giá trị của A1, B1, và C1 dựa trên chế độ
    A1.value = slider_X.get()
    if mode == "manual":
        B1.value = slider_Y.get()
        C1.value = slider_Z.get()
    elif mode == "auto":
        B1.value = x_left.value
        C1.value = x_right.value

def set_mode_manual():
    global mode
    mode = "manual"
    update_values()  # Cập nhật giá trị ngay khi thay đổi chế độ

def set_mode_auto():
    global mode
    mode = "auto"
    update_values()  # Cập nhật giá trị ngay khi thay đổi chế độ

def start_sending_commands():
    send_command.value = True  # Bắt đầu gửi lệnh sau khi bấm OK

def create_slider_interface():
    global slider_X, slider_Y, slider_Z, root  # Thêm slider_X, slider_Y, slider_Z vào global

    root = tk.Tk()
    root.title("Arduino Slider Control")
    root.geometry("800x900")  # Kích thước cửa sổ lớn hơn

    # Font lớn hơn cho nhãn và nút
    label_font = ("Helvetica", 20)
    button_font = ("Helvetica", 22, "bold")
    slider_font = ("Helvetica", 18)

    # Tạo thanh trượt cho trục X với kích thước lớn hơn
    slider_X = tk.Scale(
        root, from_=0, to=285, orient=tk.HORIZONTAL, label="X", length=600,
        font=slider_font, sliderlength=50, width=30, command=lambda x: update_values()
    )
    slider_X.pack(pady=20)

    # Tạo thanh trượt cho các trục Y và Z với kích thước lớn hơn
    slider_Y = tk.Scale(
        root, from_=0, to=180, orient=tk.HORIZONTAL, label="Y", length=600,
        font=slider_font, sliderlength=50, width=30, command=lambda y: update_values()
    )
    slider_Y.pack(pady=20)

    slider_Z = tk.Scale(
        root, from_=0, to=180, orient=tk.HORIZONTAL, label="Z", length=600,
        font=slider_font, sliderlength=50, width=30, command=lambda z: update_values()
    )
    slider_Z.pack(pady=20)

    # Chế độ chọn tự động và thủ công với nhãn lớn hơn
    mode_frame = tk.Frame(root)
    mode_frame.pack(pady=20)
    tk.Label(mode_frame, text="Chọn chế độ điều chỉnh:", font=label_font).pack(anchor="w")
    # Tạo các nút Radio để chọn chế độ
    mode_var = tk.StringVar(value="manual")  # Sử dụng StringVar để lưu trữ chế độ

    tk.Radiobutton(
        mode_frame, text="Thủ công", variable=mode_var, value="manual",
        font=label_font, command=set_mode_manual
    ).pack(anchor="w")

    tk.Radiobutton(
        mode_frame, text="Tự động", variable=mode_var, value="auto",
        font=label_font, command=set_mode_auto
    ).pack(anchor="w")


    # Nút OK để bắt đầu gửi lệnh với kích thước lớn hơn
    btn_ok = tk.Button(root, text="OK", font=button_font, command=start_sending_commands, height=2, width=10)
    btn_ok.pack(pady=30)

    root.mainloop()



def send_to_arduino():
    while True:
        if send_command.value:
            # Gửi lệnh đầu tiên ngay khi ấn nút OK
            command = f'G10 X{A1.value} Y{B1.value} Z{C1.value} E{0}\n'
            ser.write(command.encode('utf-8'))
            print("Sent:", command)

            # Đợi 3 giây trước khi bắt đầu gửi đều đặn
            time.sleep(3)
            
            # Gửi lệnh đều đặn mỗi 0.4 giây
            while send_command.value:
                if -320 < mid_x_value.value < 320 and box_detect.value == True:
                    command = f'G10 X{A1.value} Y{B1.value} Z{C1.value} A{mid_x_value.value} E{1}\n'
                else:
                    command = f'G10 X{A1.value} Y{B1.value} Z{C1.value} E{0}\n'

                ser.write(command.encode('utf-8'))
                print("Sent:", command)
                time.sleep(0.4)



# Khởi động giao diện Tkinter trong tiến trình riêng
slider_process = Process(target=create_slider_interface)
slider_process.start()



# Khởi động tiến trình gửi lệnh đến Arduino
arduino_process = Process(target=send_to_arduino)
arduino_process.start()

def moving_average(values):
    return sum(values) / len(values) if values else 0

def takeSecond(elem):
    return elem[1]


def to_sublists(lst, length = 2):
    return [lst[i:i+length] for i in range(0,(len(lst)+1-length),2)]


def rgb(img):

    img_1 = np.array(img, dtype=np.float32) / 255.0
    (b, g, r) = cv2.split(img_1)
    gray = 2 * g - 1 * b - 1 * r
    (minVal, maxVal, minLoc, maxLoc) = cv2.minMaxLoc(gray)
    gray_u8 = np.array((gray - minVal) / (maxVal - minVal) * 255, dtype=np.uint8)
    (thresh, img_2) = cv2.threshold(gray_u8, -1.0, 255, cv2.THRESH_OTSU)
    element_1 = np.array([[0, 0, 0, 1], [0, 0, 1, 0], [0, 1, 0, 0], [1, 0, 0, 0]], dtype=np.uint8)
    img_3 = cv2.erode(img_2, element_1, iterations=2)
    img_4 = cv2.dilate(img_3, element_1, iterations=4)

    return img_4


@smart_inference_mode()
def run(
    weights=ROOT / "yolov5s.pt",  # model path or triton URL
    source=ROOT / "data/images",  # file/dir/URL/glob/screen/0(webcam)
    data=ROOT / "data/coco128.yaml",  # dataset.yaml path
    imgsz=(640, 640),  # inference size (height, width)
    conf_thres=0.25,  # confidence threshold
    iou_thres=0.45,  # NMS IOU threshold
    max_det=1000,  # maximum detections per image
    device="",  # cuda device, i.e. 0 or 0,1,2,3 or cpu
    view_img=False,  # show results
    save_txt=False,  # save results to *.txt
    save_csv=False,  # save results in CSV format
    save_conf=False,  # save confidences in --save-txt labels
    save_crop=False,  # save cropped prediction boxes
    nosave=False,  # do not save images/videos
    classes=None,  # filter by class: --class 0, or --class 0 2 3
    agnostic_nms=False,  # class-agnostic NMS
    augment=False,  # augmented inference
    visualize=False,  # visualize features
    update=False,  # update all models
    project=ROOT / "runs/detect",  # save results to project/name
    name="exp",  # save results to project/name
    exist_ok=False,  # existing project/name ok, do not increment
    line_thickness=3,  # bounding box thickness (pixels)
    hide_labels=False,  # hide labels
    hide_conf=False,  # hide confidences
    half=False,  # use FP16 half-precision inference
    dnn=False,  # use OpenCV DNN for ONNX inference
    vid_stride=1,  # video frame-rate stride
):
    """
    Runs YOLOv5 detection inference on various sources like images, videos, directories, streams, etc.

    Args:
        weights (str | Path): Path to the model weights file or a Triton URL. Default is 'yolov5s.pt'.
        source (str | Path): Input source, which can be a file, directory, URL, glob pattern, screen capture, or webcam
            index. Default is 'data/images'.
        data (str | Path): Path to the dataset YAML file. Default is 'data/coco128.yaml'.
        imgsz (tuple[int, int]): Inference image size as a tuple (height, width). Default is (640, 640).
        conf_thres (float): Confidence threshold for detections. Default is 0.25.
        iou_thres (float): Intersection Over Union (IOU) threshold for non-max suppression. Default is 0.45.
        max_det (int): Maximum number of detections per image. Default is 1000.
        device (str): CUDA device identifier (e.g., '0' or '0,1,2,3') or 'cpu'. Default is an empty string, which uses the
            best available device.
        view_img (bool): If True, display inference results using OpenCV. Default is False.
        save_txt (bool): If True, save results in a text file. Default is False.
        save_csv (bool): If True, save results in a CSV file. Default is False.
        save_conf (bool): If True, include confidence scores in the saved results. Default is False.
        save_crop (bool): If True, save cropped prediction boxes. Default is False.
        nosave (bool): If True, do not save inference images or videos. Default is False.
        classes (list[int]): List of class indices to filter detections by. Default is None.
        agnostic_nms (bool): If True, perform class-agnostic non-max suppression. Default is False.
        augment (bool): If True, use augmented inference. Default is False.
        visualize (bool): If True, visualize feature maps. Default is False.
        update (bool): If True, update all models' weights. Default is False.
        project (str | Path): Directory to save results. Default is 'runs/detect'.
        name (str): Name of the current experiment; used to create a subdirectory within 'project'. Default is 'exp'.
        exist_ok (bool): If True, existing directories with the same name are reused instead of being incremented. Default is
            False.
        line_thickness (int): Thickness of bounding box lines in pixels. Default is 3.
        hide_labels (bool): If True, do not display labels on bounding boxes. Default is False.
        hide_conf (bool): If True, do not display confidence scores on bounding boxes. Default is False.
        half (bool): If True, use FP16 half-precision inference. Default is False.
        dnn (bool): If True, use OpenCV DNN backend for ONNX inference. Default is False.
        vid_stride (int): Stride for processing video frames, to skip frames between processing. Default is 1.

    Returns:
        None

    Examples:
        ```python
        from ultralytics import run

        # Run inference on an image
        run(source='data/images/example.jpg', weights='yolov5s.pt', device='0')

        # Run inference on a video with specific confidence threshold
        run(source='data/videos/example.mp4', weights='yolov5s.pt', conf_thres=0.4, device='0')
        ```
    """
    source = str(source)
    save_img = not nosave and not source.endswith(".txt")  # save inference images
    is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
    is_url = source.lower().startswith(("rtsp://", "rtmp://", "http://", "https://"))
    webcam = source.isnumeric() or source.endswith(".streams") or (is_url and not is_file)
    screenshot = source.lower().startswith("screen")

    if is_url and is_file:
        source = check_file(source)  # download

    # Directories
    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
    (save_dir / "labels" if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Set Dataloader
    bs = 1  # batch_size
    if webcam:
        view_img = check_imshow(warn=True)
        dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
        bs = len(dataset)
    elif screenshot:
        dataset = LoadScreenshots(source, img_size=imgsz, stride=stride, auto=pt)
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)

    vid_path, vid_writer = [None] * bs, [None] * bs

    # Run inference
    model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup
    seen, windows, dt = 0, [], (Profile(device=device), Profile(device=device), Profile(device=device))
    for path, im, im0s, vid_cap, s in dataset:
        with dt[0]:
            im = torch.from_numpy(im).to(model.device)
            im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim
            if model.xml and im.shape[0] > 1:
                ims = torch.chunk(im, im.shape[0], 0)

        # Inference
        with dt[1]:
            visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
            if model.xml and im.shape[0] > 1:
                pred = None
                for image in ims:
                    if pred is None:
                        pred = model(image, augment=augment, visualize=visualize).unsqueeze(0)
                    else:
                        pred = torch.cat((pred, model(image, augment=augment, visualize=visualize).unsqueeze(0)), dim=0)
                
                t1 = time_sync()  # them code
                pred = [pred, None]
            else:
                t1 = time_sync()  # them code
                pred = model(im, augment=augment, visualize=visualize)

        # Apply NMS
        with dt[2]:
            pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)
            t2 = time_sync()    # them code

        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        # Define the path for the CSV file
        csv_path = save_dir / "predictions.csv"

        # Create or append to the CSV file
        def write_to_csv(image_name, prediction, confidence):
            """Writes prediction data for an image to a CSV file, appending if the file exists."""
            data = {"Image Name": image_name, "Prediction": prediction, "Confidence": confidence}
            with open(csv_path, mode="a", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=data.keys())
                if not csv_path.is_file():
                    writer.writeheader()
                writer.writerow(data)

        # Process predictions
        for i, det in enumerate(pred):  # per image
            seen += 1

            if webcam:  # batch_size >= 1
                p, im0, frame = path[i], im0s[i].copy(), dataset.count
                s += f"{i}: "
            else:
                p, im0, frame = path, im0s.copy(), getattr(dataset, "frame", 0)

            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # im.jpg
            txt_path = str(save_dir / "labels" / p.stem) + ("" if dataset.mode == "image" else f"_{frame}")  # im.txt
            s += "%gx%g " % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            imc = im0.copy() if save_crop else im0  # for save_crop
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))

            if len(det):
                
                box_detect.value = True
                
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                for c in det[:, 5].unique():
                    n = (det[:, 5] == c).sum()  # detections per class
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string
                
                # Them code
                start = time.time()
                xyxy_list = []
                im1 = im0.copy()                
                
                # Write results
                for *xyxy, conf, cls in reversed(det):
                    c = int(cls)  # integer class
                    label = names[c] if hide_conf else f"{names[c]}"
                    confidence = float(conf)
                    confidence_str = f"{confidence:.2f}"

                    if save_csv:
                        write_to_csv(p.name, label, confidence_str)

                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
                        with open(f"{txt_path}.txt", "a") as f:
                            f.write(("%g " * len(line)).rstrip() % line + "\n")

                    if save_img or save_crop or view_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        label = None if hide_labels else (names[c] if hide_conf else f"{names[c]} {conf:.2f}")
                        annotator.box_label(xyxy, label, color=colors(c, True))
                        # annotator.box_label(xyxy, None, color=colors(c, True))
                    
                    if save_crop:
                        save_one_box(xyxy, imc, file=save_dir / "crops" / names[c] / f"{p.stem}.jpg", BGR=True)
                    
                    # Them code
                    xyxy_list.extend(xyxy)
                
            else:
                box_detect.value = False

            # Them code time of (inference + NMS)
            # print(f'{s}Done. ({t2 - t1:.3f}s)')   
            
                       

            def fast(img, x, y, draw=True):
                flag = True
                # fast = cv2.FastFeatureDetector_create(30)
                # fast.setNonmaxSuppression(False) #NMS
                # kp = fast.detect(img, None)
                # im1_with_keypoints = cv2.drawKeypoints(img, kp, None, color=(0, 255, 0), flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
                # # Hiển thị ảnh với các điểm đặc trưng

                # cv2.imshow("Keypoints on im1", im1_with_keypoints)
                # cv2.imshow("im0", im0)
                
                # left = []
                # right = []
                # kp = cv2.KeyPoint_convert(kp)
                left = []
                for point in new_lst_1:
                    if point[0] < x:
                        # Chuyển từng giá trị trong `point` sang CPU và `numpy`
                        left.append([float(p.cpu()) if isinstance(p, torch.Tensor) else p for p in point])
                
                left1 = []
                for point in new_lst:
                    if point[0] < x:
                        # Chuyển từng giá trị trong `point` sang CPU và `numpy`
                        left1.append([float(p.cpu()) if isinstance(p, torch.Tensor) else p for p in point])

                right = []
                for point in xyxy_list_sort:
                    if point[0] > x:
                        # Chuyển từng giá trị trong `point` sang CPU và `numpy`
                        right.append([float(p.cpu()) if isinstance(p, torch.Tensor) else p for p in point])

                # Chuyển đổi `left` và `right` thành mảng numpy nếu cần thiết
                left = np.array(left)
                left1 = np.array(left1)
                right = np.array(right)

                # print("Left:", left1[-1][0])
                
                global mid_x_value
                
                mid_x = int((left1[-1][0] + right[-1][0]) / 2)
                mid_y = int((left1[-1][1] + right[-1][1]) / 2)
                # print(f"Đã gửi giá trị: {mid_x}, {mid_y}")
                
                # Thêm giá trị mid_x vào danh sách và giữ kích thước tối đa là MAX_VALUES
                mid_x_values.append(mid_x)
                if len(mid_x_values) > MAX_VALUES:
                    mid_x_values.pop(0)
                
                # Tính giá trị mid_x trung bình
                filtered_mid_x = int(moving_average(mid_x_values))

                # Vẽ một điểm tại trung điểm đã được lọc nhiễu, sử dụng màu đỏ (0, 0, 255) và độ dày 5
                cv2.circle(im0, (filtered_mid_x, mid_y), 5, (0, 0, 255), -1)
                cv2.line(im0, (320,480), (mid_x,mid_y), (0, 255, 255), 3)
                
                # Tính toán và in giá trị mid_x - 320
                mid_x_value.value = filtered_mid_x - 320
                

                # Vẽ một điểm tại trung điểm, sử dụng màu đỏ (0, 0, 255) và độ dày 5
                cv2.circle(im0, (mid_x, mid_y), 10, (0, 0, 255), -1)
                

                try:
                    [vx_middle_line, vy_middle_line, x_middle_line, y_middle_line] = cv2.fitLine(left, cv2.DIST_HUBER, 0,
                                                                                                     1e-2, 1e-2)
                    k = vy_middle_line / vx_middle_line
                    if k == 0:
                        k = k + 0.0001
                    b = y_middle_line - k * x_middle_line
                except:
                    pass

                try:
                    [vx_middle_line1, vy_middle_line1, x_middle_line1, y_middle_line1] = cv2.fitLine(right,
                                        cv2.DIST_HUBER, 0, 1e-2, 1e-2)
                    k1 = vy_middle_line1 / vx_middle_line1
                    if k1 == 0:
                        k1 = k1 + 0.0001
                    b1 = y_middle_line1 - k1 * x_middle_line1
                except:
                    pass

                try:
                    x1 = (int(((y - b) / k + (y - b1) / k1) / 2))
                    y1 = y
                    x2 = (int(((480 - b) / k + (480 - b1) / k1) / 2))
                    y2 = h
                    PI = math.pi

                    if (x2 - x1) != 0:
                        k_mid = (y2 - y1) / (x2 - x1)
                    else:
                        k_mid = (y2 - y1) / 0.01

                    ceita = (PI / 2 + math.atan(k_mid)) * (180 / PI)
                    
                    # print(f"Đã gửi giá trị: {ceita}")

                    if ceita > 100:
                        ceita = abs(ceita - 180)
                    if ceita > 10:
                        flag = False
                        # flag = True

                    x11 = int((new_lst[0][0] + xyxy_list_sort[0][0]) / 6)
                    y11 = int(new_lst[0][1] / 3)
                    x22 = int((xyxy_list_sort[-1][0] + new_lst[-1][0]) / 6)
                    y22 = int(xyxy_list_sort[-1][1] / 3)
                    PI = math.pi

                    if (x22 - x11) != 0:
                        k_m = (y22 - y11) / (x22 - x11)
                    else:
                        k_m = (y22 - y11) / 0.01

                    ceita_m = (PI / 2 + math.atan(k_m)) * (180 / PI)

                    if ceita_m > 100:
                        ceita_m = abs(ceita_m - 180)

                    if ceita_m > 10:
                        flag = False
                        # flag = True
                        
                    if flag == True:
                        


                        # Tọa độ ban đầu của đường thẳng màu đỏ
                        # Đường thẳng màu đỏ 1
                        start_point_red_1 = (int((y - b) / k), y)
                        end_point_red_1 = (int((480 - b) / k), 480)
                        
                        cv2.line(im0, start_point_red_1, end_point_red_1, (0, 0, 255), 3)

                        # Đường thẳng màu đỏ 2
                        start_point_red_2 = (int((y - b1) / k1), y)
                        end_point_red_2 = (int((480 - b1) / k1), 480)
                        
                        cv2.line(im0, start_point_red_2, end_point_red_2, (0, 0, 255), 3)

                        # Đường thẳng màu vàng
                        start_point_yellow = (int(((y - b) / k + (y - b1) / k1) / 2), y)
                        end_point_yellow = (int(((480 - b) / k + (480 - b1) / k1) / 2), 480)
                        
                        
                        # cv2.line(im0, start_point_yellow, end_point_yellow, (0, 255, 255), 3)
                        
                        

                        # cv2.line(im0, (int((y - b) / k), y), (int((480 - b) / k), 480), (0, 0, 255), 3)
                        # cv2.line(im0, (int((y - b1) / k1), y), (int((480 - b1) / k1), 480), (0, 0, 255), 3)
                        # cv2.line(im0, (int(((y - b) / k + (y - b1) / k1) / 2), y),
                        #                    (int(((480 - b) / k + (480 - b1) / k1) / 2), 480),
                        #                    (0, 255, 255), 3)

                        # cv2.polylines(im0, [c], True, (255, 0, 0), 3)

                        # Vẽ đường thẳng màu vàng như trước
                        cv2.line(im0, (x1, y1), (x2, y2), (0, 255, 255), 3)
                        
                        

                        # Tính tọa độ trung điểm của đường thẳng
                        # mid_x = int((x1 + x2) / 2)
                        # mid_y = int((y1 + y2) / 2)
                        
                        
                                          
                except:
                    pass

                return im0       

            # Stream results
            im0 = annotator.result()
            if view_img:
                if platform.system() == "Linux" and p not in windows:
                    windows.append(p)
                    cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                    cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
                    cv2.setMouseCallback(str(p), onMouse)
                    # print(im0.shape[0])
                #cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond

            # Save results (image with detections)
            if save_img:
                if dataset.mode == "image":
                    cv2.imwrite(save_path, im0)
                else:  # 'video' or 'stream'
                    if vid_path[i] != save_path:  # new video
                        vid_path[i] = save_path
                        
                        if isinstance(vid_writer[i], cv2.VideoWriter):
                            vid_writer[i].release()  # release previous video writer
                        
                        if vid_cap:  # video
                            fps = vid_cap.get(cv2.CAP_PROP_FPS)
                            w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        else:  # stream
                            fps, w, h = 30, im0.shape[1], im0.shape[0]
                        
                        save_path = str(Path(save_path).with_suffix(".mp4"))  # force *.mp4 suffix on results videos
                        vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))                

                    # them code
                    h0 = im0.shape[0]
                    w0 = im0.shape[1]

                    im0 = cv2.resize(im0, (640, 480))
                    im1 = cv2.resize(im1, (640, 480))
                    xyxy_list_sort = to_sublists(xyxy_list)
                    
                    new_lst = []
                    for i in range(len(xyxy_list_sort)):
                        if i % 2 == 0:
                            new_lst.append(xyxy_list_sort[i])

                    new_lst.sort(key=takeSecond)
                    for i in range(len(xyxy_list_sort)):
                        for j in xyxy_list_sort:
                            if j in new_lst:
                                xyxy_list_sort.remove(j)

                    xyxy_list_sort.sort(key=takeSecond)
                    
                    new_lst_1 = [[new_lst[i][0], xyxy_list_sort[i][1]] for i in range(len(new_lst))]
                    
                    if len(new_lst_1) > 2:
                        new_lst_1 = new_lst_1[-2:]  # Chỉ lấy 2 phần tử cuối cùng
                    
                    if len(new_lst) > 2:
                        new_lst = new_lst[-2:]  # Chỉ lấy 2 phần tử cuối cùng
                        
                    if len(xyxy_list_sort) > 2:
                        xyxy_list_sort = xyxy_list_sort[-2:]  # Chỉ lấy 2 phần tử cuối cùng    
                    
                        
                    
                    # print(f"Đã gửi giá trị: {new_lst_1}")

                    c = np.array(
                        [[int(new_lst_1[0][0] / (h0 / 480)), int(new_lst_1[0][1]) / (h0 / 480)],
                         [int(xyxy_list_sort[0][0] / (h0 / 480)),
                          int(new_lst_1[0][1]) / (h0 / 480)],
                         [int(xyxy_list_sort[-1][0] / (w0 / 640)), int(xyxy_list_sort[-1][1] / (w0 / 640))],
                         [int(new_lst_1[-1][0] / (w0 / 640)), int(xyxy_list_sort[-1][1] / (w0 / 640))]], np.int32)
                    

                    # print(f"Đã gửi giá trị: {c}")
                    # for point in c:
                    #     cv2.circle(im1, (point[0], point[1]), radius=5, color=(0, 0, 255), thickness=-1)  # Màu đỏ
                    
                    for point in new_lst_1:
                        cv2.circle(im0, (int(point[0]), int(point[1])), radius=5, color=(0, 255, 0), thickness=-1)  # Màu xanh lá cây

                    # Vẽ các điểm trong xyxy_list_sort màu đỏ
                    for point in xyxy_list_sort:
                        cv2.circle(im0, (int(point[0]), int(point[1])), radius=5, color=(0, 0, 255), thickness=-1)  # Màu đỏ
                    
                    cv2.line(im0, (320, 0), (320, 480), (255, 0, 0), 1)
                    
                    # mid_x = int((new_lst[-1][0] + xyxy_list_sort[-1][0]) / 2)
                    # mid_y = int((new_lst[-1][1] + xyxy_list_sort[-1][1]) / 2)
                    
                    # cv2.line(im0, (320,480), (mid_x,mid_y), (0, 255, 255), 3)
                    # cv2.circle(im0, (mid_x, mid_y), 10, (0, 0, 255), -1)

                    roi_mask = np.zeros((480, 640), dtype=np.uint8)
                    cv2.fillPoly(roi_mask, [c], 255)
                    roi = cv2.bitwise_and(im0, im0, mask=roi_mask)
                    # roi = rgb(roi)
                    
                    # im0 = rgb(im0)

                    fast(roi, int((new_lst_1[0][0] + xyxy_list_sort[0][0]) / (w0 / 320)), int(new_lst_1[0][1] / (h0 / 480)))

                    im0 = cv2.resize(im0, (640, 480))
                    im1 = cv2.resize(im1, (640, 480))
                    # print('###################')
                    
                    end = time.time()
                    im0 = cv2.flip(im0, 1)
                    cv2.imshow(str(p), im0)
                    # cv2.imshow("im1", im1)
                    # cv2.imshow("roi", roi)

                    
                    if cv2.waitKey(1) == ord('q'):
                        break
                    
                    # vid_writer[i].write(im0)

        # Print time (inference-only)
        # LOGGER.info(f"{s}{'' if len(det) else '(no detections), '}{dt[1].dt * 1E3:.1f}ms")

    # Print results
    # t = tuple(x.t / seen * 1e3 for x in dt)  # speeds per image
    # LOGGER.info(f"Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}" % t)
    if save_txt or save_img:
        s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ""
        LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
    
    if update:
        strip_optimizer(weights[0])  # update model (to fix SourceChangeWarning)
    
    # them code
    # print(f'Done. ({end - start}s)')


def parse_opt():
    """
    Parse command-line arguments for YOLOv5 detection, allowing custom inference options and model configurations.

    Args:
        --weights (str | list[str], optional): Model path or Triton URL. Defaults to ROOT / 'yolov5s.pt'.
        --source (str, optional): File/dir/URL/glob/screen/0(webcam). Defaults to ROOT / 'data/images'.
        --data (str, optional): Dataset YAML path. Provides dataset configuration information.
        --imgsz (list[int], optional): Inference size (height, width). Defaults to [640].
        --conf-thres (float, optional): Confidence threshold. Defaults to 0.25.
        --iou-thres (float, optional): NMS IoU threshold. Defaults to 0.45.
        --max-det (int, optional): Maximum number of detections per image. Defaults to 1000.
        --device (str, optional): CUDA device, i.e., '0' or '0,1,2,3' or 'cpu'. Defaults to "".
        --view-img (bool, optional): Flag to display results. Defaults to False.
        --save-txt (bool, optional): Flag to save results to *.txt files. Defaults to False.
        --save-csv (bool, optional): Flag to save results in CSV format. Defaults to False.
        --save-conf (bool, optional): Flag to save confidences in labels saved via --save-txt. Defaults to False.
        --save-crop (bool, optional): Flag to save cropped prediction boxes. Defaults to False.
        --nosave (bool, optional): Flag to prevent saving images/videos. Defaults to False.
        --classes (list[int], optional): List of classes to filter results by, e.g., '--classes 0 2 3'. Defaults to None.
        --agnostic-nms (bool, optional): Flag for class-agnostic NMS. Defaults to False.
        --augment (bool, optional): Flag for augmented inference. Defaults to False.
        --visualize (bool, optional): Flag for visualizing features. Defaults to False.
        --update (bool, optional): Flag to update all models in the model directory. Defaults to False.
        --project (str, optional): Directory to save results. Defaults to ROOT / 'runs/detect'.
        --name (str, optional): Sub-directory name for saving results within --project. Defaults to 'exp'.
        --exist-ok (bool, optional): Flag to allow overwriting if the project/name already exists. Defaults to False.
        --line-thickness (int, optional): Thickness (in pixels) of bounding boxes. Defaults to 3.
        --hide-labels (bool, optional): Flag to hide labels in the output. Defaults to False.
        --hide-conf (bool, optional): Flag to hide confidences in the output. Defaults to False.
        --half (bool, optional): Flag to use FP16 half-precision inference. Defaults to False.
        --dnn (bool, optional): Flag to use OpenCV DNN for ONNX inference. Defaults to False.
        --vid-stride (int, optional): Video frame-rate stride, determining the number of frames to skip in between
            consecutive frames. Defaults to 1.

    Returns:
        argparse.Namespace: Parsed command-line arguments as an argparse.Namespace object.

    Example:
        ```python
        from ultralytics import YOLOv5
        args = YOLOv5.parse_opt()
        ```
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", nargs="+", type=str, default=ROOT / "yolov5s.pt", help="model path or triton URL")
    parser.add_argument("--source", type=str, default=ROOT / "data/images", help="file/dir/URL/glob/screen/0(webcam)")
    parser.add_argument("--data", type=str, default=ROOT / "data/coco128.yaml", help="(optional) dataset.yaml path")
    parser.add_argument("--imgsz", "--img", "--img-size", nargs="+", type=int, default=[640], help="inference size h,w")
    parser.add_argument("--conf-thres", type=float, default=0.25, help="confidence threshold")
    parser.add_argument("--iou-thres", type=float, default=0.45, help="NMS IoU threshold")
    parser.add_argument("--max-det", type=int, default=1000, help="maximum detections per image")
    parser.add_argument("--device", default="", help="cuda device, i.e. 0 or 0,1,2,3 or cpu")
    parser.add_argument("--view-img", action="store_true", help="show results")
    parser.add_argument("--save-txt", action="store_true", help="save results to *.txt")
    parser.add_argument("--save-csv", action="store_true", help="save results in CSV format")
    parser.add_argument("--save-conf", action="store_true", help="save confidences in --save-txt labels")
    parser.add_argument("--save-crop", action="store_true", help="save cropped prediction boxes")
    parser.add_argument("--nosave", action="store_true", help="do not save images/videos")
    parser.add_argument("--classes", nargs="+", type=int, help="filter by class: --classes 0, or --classes 0 2 3")
    parser.add_argument("--agnostic-nms", action="store_true", help="class-agnostic NMS")
    parser.add_argument("--augment", action="store_true", help="augmented inference")
    parser.add_argument("--visualize", action="store_true", help="visualize features")
    parser.add_argument("--update", action="store_true", help="update all models")
    parser.add_argument("--project", default=ROOT / "runs/detect", help="save results to project/name")
    parser.add_argument("--name", default="exp", help="save results to project/name")
    parser.add_argument("--exist-ok", action="store_true", help="existing project/name ok, do not increment")
    parser.add_argument("--line-thickness", default=3, type=int, help="bounding box thickness (pixels)")
    parser.add_argument("--hide-labels", default=False, action="store_true", help="hide labels")
    parser.add_argument("--hide-conf", default=False, action="store_true", help="hide confidences")
    parser.add_argument("--half", action="store_true", help="use FP16 half-precision inference")
    parser.add_argument("--dnn", action="store_true", help="use OpenCV DNN for ONNX inference")
    parser.add_argument("--vid-stride", type=int, default=1, help="video frame-rate stride")
    opt = parser.parse_args()
    opt.imgsz *= 2 if len(opt.imgsz) == 1 else 1  # expand
    print_args(vars(opt))
    return opt


def main(opt):
    """
    Executes YOLOv5 model inference based on provided command-line arguments, validating dependencies before running.

    Args:
        opt (argparse.Namespace): Command-line arguments for YOLOv5 detection. See function `parse_opt` for details.

    Returns:
        None

    Note:
        This function performs essential pre-execution checks and initiates the YOLOv5 detection process based on user-specified
        options. Refer to the usage guide and examples for more information about different sources and formats at:
        https://github.com/ultralytics/ultralytics

    Example usage:

    ```python
    if __name__ == "__main__":
        opt = parse_opt()
        main(opt)
    ```
    """
    check_requirements(ROOT / "requirements.txt", exclude=("tensorboard", "thop"))
    run(**vars(opt))


if __name__ == "__main__":
    opt = parse_opt()
    main(opt)
