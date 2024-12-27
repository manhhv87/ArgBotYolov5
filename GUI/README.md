Cách chạy:
Bước 1: Tải các thư viện cần thiết.
    Nvidia và Cuda
        https://www.notion.so/NVIDIA-CUDA-Toolkit-c82511f0930b4140b046f55a37d26cd0
    Anaconda
        https://www.notion.so/Anaconda-fd2733ff9dbb4e7a80c4bf8fdabf9737
    Python bản 3.12.7 (bản thấp hơn lỗi)
    Tạo môi trường mới:
        conda create -n pytorch-dev python=3.12 -y
    Các thư viện cần thiết cho Yolov5:
        pip install -r requirements.txt
    Gỡ opencv-python, cài opencv-python-headless
        pip uninstall opencv-python
        pip install opencv-python-headless
    Thư viện pyserial
        pip install pyserial
    Pyqt
        pip install pyqt5
        pip install pyqt5-tools
    Các thư viện khác tự cài ........... :>

Bước 2: Chạy file gui_handle.py

NOTE: Khử méo ảnh trong file utils/dataloader.py dòng 351
