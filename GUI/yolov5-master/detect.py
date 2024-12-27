import argparse
import sys
import time
import torch
import cv2
from pathlib import Path
from models.common import DetectMultiBackend
from utils.dataloaders import LoadStreams
from utils.general import (non_max_suppression, scale_boxes, check_img_size, check_imshow)
from utils.torch_utils import select_device

# Cấu hình và khởi tạo model YOLOv5
def run(weights='best.pt', source=2, imgsz=(640, 640), conf_thres=0.25, iou_thres=0.45, device=''):
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # Kiểm tra kích thước hình ảnh đầu vào

    # Khởi tạo video stream từ webcam 2
    dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt)

    model.warmup(imgsz=(1, 3, *imgsz))  # Khởi động model
    for path, im, im0s, vid_cap, s in dataset:
        im = torch.from_numpy(im).to(device)
        im = im.float() / 255.0  # Chuyển đổi sang định dạng [0, 1]
        if len(im.shape) == 3:
            im = im[None]  # Thêm batch dimension nếu cần

        # Thực hiện suy luận
        pred = model(im)
        pred = non_max_suppression(pred, conf_thres, iou_thres, max_det=1000)

        for det in pred:  # Duyệt qua từng khung hình
            im0 = im0s.copy()

            if len(det):
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()
                for *xyxy, conf, cls in reversed(det):
                    x1, y1, x2, y2 = map(int, xyxy)
                    confidence = float(conf)
                    class_id = int(cls)
                    label = f"{names[class_id]} {confidence:.2f}"
                    
                    # Vẽ bounding box và nhãn lên khung hình
                    cv2.rectangle(im0, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(im0, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            # Hiển thị kết quả
            cv2.imshow("YOLOv5 Detection", im0)
            if cv2.waitKey(1) == ord('q'):  # Nhấn 'q' để thoát
                break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--weights', type=str, default='best.pt', help='model path')
    parser.add_argument('--source', type=int, default=2, help='camera index')
    parser.add_argument('--imgsz', type=int, nargs='+', default=[640, 640], help='inference size (h, w)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='NMS IoU threshold')
    parser.add_argument('--device', default='', help='cuda device or cpu')
    opt = parser.parse_args()
    
    # Chạy chương trình
    run(opt.weights, opt.source, opt.imgsz, opt.conf_thres, opt.iou_thres, opt.device)
