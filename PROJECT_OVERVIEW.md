# Project Overview – Hệ thống phân loại rác tái chế sử dụng AI

## 1) Mục tiêu dự án
Dự án xây dựng một hệ thống phân loại rác tái chế theo thời gian thực bằng thị giác máy tính và robot gắp–thả. Hệ thống nhận diện vật thể trên băng tải, quy đổi tọa độ ảnh sang tọa độ làm việc của robot, sau đó điều khiển robot thực hiện chu trình gắp và thả theo loại vật.

## 2) Kiến trúc tổng thể
Hệ thống gồm 3 khối chính:

- **GUI desktop (PyQt6)**
  - Giao diện vận hành (Start/Stop/Reset, Connect/Disconnect, trạng thái robot/hệ thống).
  - Nhận ảnh webcam, chạy mô hình AI, hiển thị bounding box và bảng dữ liệu đối tượng.
  - Giao tiếp Serial với ESP32 chính bằng khung lệnh dạng `<...>`.

- **ESP32 chính – Robot Arm (`ESP_ROBOT_ARM`)**
  - Điều khiển cơ cấu tay máy (PID theo trục, homing, trajectory planning).
  - Xử lý động học thuận/nghịch để chuyển từ tọa độ điểm đích sang lệnh chuyển động.
  - Thực thi chu trình pick-and-place nhiều bước (điểm hút → nâng/hạ → điểm thả theo loại vật → về home).
  - Đồng bộ với ESP32 phụ và phản hồi trạng thái về GUI.

- **ESP32 phụ – Băng tải & I/O (`ESP_SUB`)**
  - Điều khiển băng tải, relay, nút cứng (ON/OFF/ALARM/RESET).
  - Gửi trạng thái start/stop/alarm/reset về ESP32 chính qua Serial2.

## 3) Công nghệ sử dụng

- **Desktop/AI**: Python, PyQt6, OpenCV, NumPy, PySerial.
- **Nhận diện đối tượng**: Ultralytics YOLO (2 lớp: `can`, `plastic-bottle`).
- **Theo dõi đối tượng**: Deep SORT (đã tích hợp loader trong code).
- **Firmware**: Arduino framework trên ESP32 (PlatformIO), BasicLinearAlgebra, AS5600 library.

## 4) Luồng dữ liệu vận hành

1. Camera ghi hình băng tải trong GUI.
2. YOLO nhận diện và xác định tâm đối tượng.
3. GUI quy đổi pixel → tọa độ thực bằng ma trận homography.
4. GUI gửi lệnh tọa độ và loại vật sang ESP32 chính qua Serial.
5. ESP32 chính tính quỹ đạo + điều khiển robot gắp/thả.
6. ESP32 chính trao đổi trạng thái với ESP32 phụ để điều phối băng tải.
7. Trạng thái robot/hệ thống được gửi ngược về GUI để hiển thị realtime.

## 5) Cấu trúc thư mục chính

- `GUI_QT/`: giao diện và pipeline AI/camera/serial.
- `CODE_ESP32/ESP_ROBOT_ARM/`: firmware tay máy (động học, điều khiển, truyền thông).
- `CODE_ESP32/ESP_SUB/`: firmware băng tải và I/O an toàn.
- `hardware/`: tài liệu phần cứng, datasheet, file tham khảo cơ khí.
- `best_can_plastic_bottle_final.pt`: trọng số mô hình nhận diện.

## 6) Giao thức lệnh Serial (khái quát)

- **Từ GUI đến ESP32 chính**: start/stop/reset, enable/disable, set home, gửi tọa độ/loại vật.
- **Từ ESP32 chính về GUI**: vị trí trục (`x,y,z`), trạng thái cho phép gắp (`g`), trạng thái robot (`r`), trạng thái hệ thống (`s`), tín hiệu reset camera (`d`).
- **Giữa ESP32 chính và ESP32 phụ**: đồng bộ start/stop/reset/alarm cho băng tải.

## 7) Điểm mạnh của hệ thống

- Tách module rõ ràng giữa GUI, AI, điều khiển robot và băng tải.
- Có phản hồi trạng thái hai chiều, thuận tiện vận hành thực nghiệm.
- Triển khai đầy đủ chuỗi từ nhận diện → quy đổi tọa độ → điều khiển chấp hành.

## 8) Lưu ý khi chạy lại dự án

- Cập nhật lại đường dẫn model YOLO trong code GUI cho đúng môi trường hiện tại.
- Kiểm tra COM port và baudrate trước khi Connect.
- Đồng bộ phiên bản thư viện Python theo `GUI_QT/requirements.txt`.
- Nạp đúng firmware cho từng board (`ESP_ROBOT_ARM` và `ESP_SUB`) trước khi chạy tích hợp.
