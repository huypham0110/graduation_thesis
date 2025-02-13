import sys

from PyQt6.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt6.QtCore import pyqtSlot, QThread, pyqtSignal, Qt, QTimer, QRect
from PyQt6 import QtGui, QtWidgets
from PyQt6.QtGui import QPixmap, QImage, QGuiApplication, QPen, QPainter, QColor

from new_gui import Ui_MainWindow
import warnings
import serial
from serial.tools import list_ports
import cv2
import numpy as np
from ultralytics import YOLO
import time
from deep_sort_realtime.deepsort_tracker import DeepSort
from collections import deque

QGuiApplication.setHighDpiScaleFactorRoundingPolicy(Qt.HighDpiScaleFactorRoundingPolicy.Floor)

warnings.filterwarnings("ignore", category=DeprecationWarning)

class_names = ['can', 'plastic-bottle']
old_center_xx = 0

H = np.array([[-0.194979026, 1.13785263*10, 1.82080638*10*10],
              [1.06170995*10, 0.127216300,  -4.63231577*10*10*10],
              [8.08147919*0.000001, 2.74915612*0.0001, 1]], dtype=np.float64)

class gui_final(QMainWindow):
    def __init__(self):
        super().__init__()
        self.uic = Ui_MainWindow()       
        self.uic.setupUi(self) 
        self.showMaximized()
        
        self.serial_connection = None
        self.refresh_ports()
        self.thread = {}
        self.homeDone = False
        self.data_queue = deque()

        ### Nut nhan ###
        self.uic.butConnect.clicked.connect(self.nhanConnect)
        self.uic.butDisconnect.clicked.connect(self.nhanDisconnect)
        self.uic.butOnWebcam.clicked.connect(self.onWebcam)
        self.uic.butOffWebcam.clicked.connect(self.offWebcam)
        self.uic.butStart.clicked.connect(self.butStart)
        self.uic.butStop.clicked.connect(self.butStop)
        self.uic.butReset.clicked.connect(self.butReset)
        self.uic.butEnable.clicked.connect(self.butEnable)
        self.uic.butDisable.clicked.connect(self.butDisable)
        self.uic.butHome.clicked.connect(self.butHome)
        self.uic.butResetCom.clicked.connect(self.refresh_ports)

        self.uic.actionControl.triggered.connect(self.butControl)
        self.uic.actionIntroduction.triggered.connect(self.butIntroduction)

    def butControl(self):
        self.uic.stackedWidget.setCurrentWidget(self.uic.page_2)
    def butIntroduction(self):
        self.uic.stackedWidget.setCurrentWidget(self.uic.page)

    ### Control system ###
    # Nut start
    def butStart(self):
        try:
            self.thread['esp32'].sendData('<a>')
            self.uic.statusbar.showMessage("Đã nhấn Start",5000)  
        except:
            self.uic.statusbar.showMessage("Vui lòng kết nối với ESP trước",5000)
    
    # Nut stop
    def butStop(self):
        try:
            self.thread['esp32'].sendData('<b>')
            self.uic.statusbar.showMessage("Đã nhấn Stop",5000)  
        except:
            self.uic.statusbar.showMessage("Vui lòng kết nối với ESP trước",5000)

        ##### Dung detect
        try:
            self.thread[1].reset()
        except:
            self.uic.statusbar.showMessage("Webcam đang tắt",5000) 
        self.offWebcam()

    # Nut reset
    def butReset(self):
        try:
            self.thread['esp32'].sendData('<r>')
            self.uic.statusbar.showMessage("Đã nhấn Reset",5000)  
        except:
            self.uic.statusbar.showMessage("Vui lòng kết nối với ESP trước",5000)    

        ##### Reset yolo
        try:
            self.thread[1].reset()
        except:
            self.uic.statusbar.showMessage("Webcam đang tắt",5000) 

        self.uic.tableWidget.clearContents()
        self.uic.tableWidget.setRowCount(0)
        self.data_queue.clear()   
    
    ### Control robot ###
    # Nut enable robot
    def butEnable(self):
        try:
            self.thread['esp32'].sendData('<f0>')
            self.uic.statusbar.showMessage("Đã nhấn Enable Robot",5000)
            self.uic.butEnable.setEnabled(False)  
            self.uic.butDisable.setEnabled(True)  
        except:
            self.uic.statusbar.showMessage("Vui lòng kết nối với ESP trước",5000)  
    
    # Nut disable robot
    def butDisable(self):
        try:
            self.thread['esp32'].sendData('<f1>')
            self.uic.statusbar.showMessage("Đã nhấn Disable Robot",5000)  
            self.uic.butEnable.setEnabled(True)  
            self.uic.butDisable.setEnabled(False)  
        except:
            self.uic.statusbar.showMessage("Vui lòng kết nối với ESP trước",5000)     
    
    # Nut set home
    def butHome(self):
        try:
            self.thread['esp32'].sendData('<h>')
            self.uic.statusbar.showMessage("Đã nhấn Set Home",5000)  
        except:
            self.uic.statusbar.showMessage("Vui lòng kết nối với ESP trước",5000)     

    # Nut test robot
    def butTest(self):
        px = float(self.uic.labPx.text()) * 10
        py = float(self.uic.labPy.text()) * 10
        pz = float(self.uic.labPz.text()) * 10
        try:
            self.thread['esp32'].sendData('<m'+str(px)+'>'+'<n'+str(py)+'>'+'<p'+str(pz)+'><g>')
            self.uic.statusbar.showMessage("Da nhan Test Robot",5000)  
        except:
            self.uic.statusbar.showMessage("Vui lòng kết nối với ESP trước",5000)
    
    ### Camera, AI config ###
    # Nut bat webcam
    def onWebcam(self):
        self.thread[1] = capture_video(index=1, data_queue=self.data_queue)
        self.thread[1].start()

        self.thread[1].signal.connect(self.show_webcam)
        self.thread[1].errol_signal.connect(self.show_errol)
        self.thread[1].data_send.connect(self.sendData)
        self.thread[1].update_table_signal.connect(self.update_table_signal)

        self.uic.butOnWebcam.setEnabled(False)
        self.uic.butOffWebcam.setEnabled(True)

    # Hien thi webcam
    def show_webcam(self, cv_img):
        """Updates the image_label with a new opencv image"""
        qt_img = self.convert_cv_qt(cv_img)
        self.uic.labWebcam.setPixmap(qt_img)
    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format.Format_RGB888)
        return QPixmap.fromImage(convert_to_Qt_format)

    # Thong bao loi tu thread Webcam
    def show_errol(self, message_errol):
        self.uic.statusbar.showMessage(message_errol,5000)

    # Gui toa do ve ESP32 tu AI
    def px_to_mm(self, px, py, H):
        pixel_coords = np.array([px, py, 1], dtype=np.float64) # Toa do pixel (x, y, 1)
        real_coords = np.dot(H, pixel_coords) # Chuyen doi bang phep nhan ma tran
        real_coords /= real_coords[2] # Chuan hoa (homogeneous to Cartesian)
        return real_coords[:2]
    def sendData(self, data_id, data_x, data_y):
        self.rb_coor = self.px_to_mm(data_x,data_y,H)
        rb_x = float(self.rb_coor[0])
        rb_y = float(self.rb_coor[1])
        rb_z = (self.uic.labPzInput.text())
        if self.uic.labPzInput.text() == 'NONE':
            rb_z = float(-800)
        try:
            self.thread['esp32'].sendData("<q><v{}><x{:.3f}><y{:.3f}><z{}>".format(data_id,rb_x-100,rb_y+100,rb_z))
            self.uic.statusbar.showMessage("<q><v{}><x{:.3f}><y{:.3f}><z{}>".format(data_id,rb_x,rb_y,rb_z),5000)
        except:
            self.uic.statusbar.showMessage("Lỗi chưa mở kết nối với ESP32",5000)
    
    # Update table
    def update_table_signal(self):
        self.uic.tableWidget.clearContents()
        self.uic.tableWidget.setRowCount(0)
        if not self.data_queue:
            return
        for data in self.data_queue:
            class_id, center_x, center_y = data
            row_position = 0
            self.uic.tableWidget.insertRow(row_position)    
            self.uic.tableWidget.setItem(row_position, 0, QtWidgets.QTableWidgetItem(class_names[int(class_id)]))
            self.uic.tableWidget.setItem(row_position, 1, QtWidgets.QTableWidgetItem(str(center_x)))
            self.uic.tableWidget.setItem(row_position, 2, QtWidgets.QTableWidgetItem(str(center_y)))

    # Nut tat Webcam
    def offWebcam(self):
        try:
            self.thread[1].stop()
        except:
            self.uic.statusbar.showMessage("Webcam đang tắt",5000)     
        self.uic.butOnWebcam.setEnabled(True)
        self.uic.butOffWebcam.setEnabled(False)

    ### ESP32 Connection ###
    # Nut refresh port
    def refresh_ports(self):
        self.uic.cbCom.clear()
        ports = list_ports.comports()
        for port in ports:
            self.uic.cbCom.addItem(port.device)

    # Nut connect ESP32
    @pyqtSlot()
    def nhanConnect(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.uic.statusbar.showMessage("Đã kết nối, vui lòng Ngắt kết nối trước khi Kết nối lại ",5000)
            return
        port = self.uic.cbCom.currentText()
        br = self.uic.cbBr.currentText()
        try:
            self.serial_connection = serial.Serial(port=port,baudrate=int(br),timeout=1)

            self.uic.statusbar.showMessage("Kết nối thành công",5000)
            self.uic.labConnection.setText("CONNECTED")
            self.uic.labConnection.setStyleSheet("color: #22c930;font-weight: bold;")
        except serial.SerialException as e:
            self.uic.statusbar.showMessage("Lỗi kết nối Serial",5000)

        self.thread['esp32'] = ESP32Thread(serial_connection=self.serial_connection)
        self.thread['esp32'].start()

        self.thread['esp32'].data_received.connect(self.handle_esp32_data)
        self.thread['esp32'].status_message.connect(self.show_status)
    
    # Xu ly du lieu nhan tu ESP32
    def handle_esp32_data(self, command, data):
        if(command == 'd'):
            try:
                self.thread[1].handle_esp32_data_cam(data)
            except:
                self.uic.statusbar.showMessage("Webcam đang tắt",5000)

        if(command == 'g'):
            if(data=='1'):
                self.uic.labWorking.setText("DONE")
                self.uic.labWorking.setStyleSheet("color: #22c930;font-weight: bold;")
            if(data=='0' and self.uic.labPx.text() == 'NONE'):
                self.uic.labWorking.setText("NONE")
                self.uic.labWorking.setStyleSheet("color: #ad1010;font-weight: bold;")
            elif(data=='0'): 
                self.uic.labWorking.setText("IS PICKING")
                self.uic.labWorking.setStyleSheet("color: #ad1010;font-weight: bold;")
            try:
                self.thread[1].handle_esp32_data_cam(data)
            except:
                self.uic.statusbar.showMessage("Webcam đang tắt",5000)

        if(command == 'r'):
            if(data=='1'):
                self.uic.labRobotStatus.setText("WORKING")
                self.uic.labRobotStatus.setStyleSheet("color: #22c930;font-weight: bold;")
            if(data=='0'):
                self.uic.labRobotStatus.setText("OFF")
                self.uic.labRobotStatus.setStyleSheet("color: #ad1010;font-weight: bold;")

        if(command == 's'):
            if(data=='1'):
                self.uic.labSystemStatus.setText("WORKING")
                self.uic.labSystemStatus.setStyleSheet("color: #22c930;font-weight: bold;")
            if(data=='0'):
                self.uic.labSystemStatus.setText("OFF")
                self.uic.labSystemStatus.setStyleSheet("color: #ad1010;font-weight: bold;")  
            if(data=='2'):
                self.uic.labSystemStatus.setText("EMERGENCY !!!")
                self.uic.labSystemStatus.setStyleSheet("color: #ad1010;font-weight: bold;")               

        if(command == 'x'):
            if(data=="0.00"):
                self.homeDone = False
            else:
                self.homeDone = True
                self.uic.labPx.setText(data)
        if(self.homeDone == True and command == 'y'):
            self.uic.labPy.setText(data)
        if(self.homeDone == True and command == 'z'):
            self.uic.labPz.setText(data)
        if(self.homeDone == False):
            self.uic.labPx.setText("NONE")
            self.uic.labPy.setText("NONE")
            self.uic.labPz.setText("NONE")

        if(command == 'e'):
            try:
                self.thread[1].reset()
            except:
                self.uic.statusbar.showMessage("Webcam đang tắt",5000) 

            self.uic.tableWidget.clearContents()
            self.uic.tableWidget.setRowCount(0)
            self.data_queue.clear() 

            self.uic.butEnable.setEnabled(False)
            self.uic.butDisable.setEnabled(True)
            self.uic.butHome.setEnabled(True)
            self.uic.butStart.setEnabled(True)
            self.uic.butStop.setEnabled(False)
            self.uic.butReset.setEnabled(True)
            
        if(command == 'a'):
            self.uic.statusbar.showMessage("Đã nhấn Start",5000)  

        if(command == 'b'):
            self.uic.statusbar.showMessage("Đã nhấn Stop",5000)  
            self.uic.butReset.setEnabled(True)
            self.uic.butEnable.setEnabled(True)
            self.uic.butDisable.setEnabled(True)
            self.uic.butHome.setEnabled(True)

        if(command == 'l'):
            self.uic.statusbar.showMessage("Emergency !!!",5000)  
            self.uic.butStart.setEnabled(False)
            self.uic.butStop.setEnabled(False)
            self.uic.butReset.setEnabled(False)
            self.uic.butEnable.setEnabled(False)
            self.uic.butDisable.setEnabled(False)
            self.uic.butHome.setEnabled(False)
            try:
                self.thread[1].reset()
            except:
                self.uic.statusbar.showMessage("Webcam đang tắt",5000) 
            self.offWebcam()

    # Thong bao trang thai tu thread ESP32
    def show_status(self, stt):
        self.uic.statusbar.showMessage(stt,5000)

    # Nut disconnect ESP32
    @pyqtSlot()
    def nhanDisconnect(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.thread['esp32'].stop()
            self.serial_connection.close()

            self.uic.labConnection.setText("DISCONNECTED")
            self.uic.labConnection.setStyleSheet("color: #ad1010;font-weight: bold;")
            self.uic.statusbar.showMessage("Ngắt kết nối thành công",5000)

class ESP32Thread(QThread):
    data_received = pyqtSignal(str,str)
    status_message = pyqtSignal(str)

    def __init__(self, serial_connection):
        super().__init__()
        self.serial_connection = serial_connection
        self.running = True

    def run(self):
        buffer = ''
        while self.running:
            if self.serial_connection and self.serial_connection.is_open:
                try:
                    buffer += self.serial_connection.read().decode()
                    if len(buffer) > 1024:
                        buffer = buffer[-1024:]

                    # Xu li cac cap du lieu
                    while '<' in buffer and '>' in buffer:
                        start = buffer.find('<')
                        end = buffer.find('>')
                        if start < end:
                            data = buffer[start+1:end]
                            command = data[0]
                            value = data[1:]
                            self.data_received.emit(command, value)
                            buffer = buffer[end+1:]
                        else:
                            buffer = buffer[start+1:]
                except Exception as e:
                    self.status_message.emit(f"Lỗi đọc dữ liệu: {e}")
                    self.running = False
            else:
                self.status_message.emit("Kết nối Serial đã bị ngắt")
                self.running = False

    # Nhan du lieu sau detect va gui ve ESP32
    def sendData(self, data):
        if self.serial_connection and self.serial_connection.is_open:
            if data:
                try:
                    self.serial_connection.write(data.encode())
                except Exception as e:
                    self.status_message.emit(f"Lỗi khi gửi dữ liệu: {str(e)}",5000)
            else:
                self.status_message.emit("Chưa có dữ liệu",5000)
        else:
            self.status_message.emit("Chưa kết nối Serial",5000)

    def stop(self):
        self.running = False
        self.terminate()
        self.wait()


class capture_video(QThread):
    signal = pyqtSignal(np.ndarray)
    errol_signal = pyqtSignal(str)
    data_send = pyqtSignal(int,int,int)
    update_table_signal = pyqtSignal()
    
    def __init__(self, index, data_queue):
        self.index = index
        self.running = True
        self.esp32_data = None
        self.data_queue = data_queue
        self.roi = (31, 131, 600, 316)
        self.frame_id = 0

        print("start threading", self.index)
        super(capture_video, self).__init__()

    def run(self):      
        self.model = self.load_model()
        self.tracker = self.load_tracker()
        self.device = 'cuda'
        self.tracks = []
        self.detects = []
        self.player=None
        self.counter = []
        self.old_count = []
        self.mangdem = []
        self.run_program()
    
    def run_program(self):
        self.player = cv2.VideoCapture(0)
        if not self.player.isOpened():
            self.errol_signal.emit("Không thể đọc camera")
            return
        
        while self.running:
            start_time = time.time()
            ret, frame = self.player.read()
            if not ret:
                continue
            
            # truyen vao model de detect
            results = self.model(frame,agnostic_nms=True)
            result = results[0]

            self.detects = []

            boxes = result.boxes.xyxy.cpu().numpy() # Bounding boxes in (x1, y1, x2, y2) format
            confidences = result.boxes.conf.cpu().numpy()  # Confidence scores
            class_labels = result.boxes.cls.cpu().numpy()

            for box, confidence, class_label in zip(boxes, confidences, class_labels):
                print("class label: ",class_label)
                print("conf: ",confidence)
                if (class_label != 0 and class_label != 1) or confidence < 0.6:
                        continue
                x1, y1, x2, y2 = map(int, box)
                center_xx = int((x1+x2)/2)
                center_yy = int((y1+y2)/2)
                if center_xx > 190 and center_xx < 210 and center_yy >125 and center_yy<316:
                    if self.esp32_data == '1':
                        self.data_queue.append((int(class_label), center_xx, center_yy))
                        self.update_table_signal.emit()
                        self.data_send.emit(int(class_label),center_xx, center_yy)
                        self.esp32_data = None
                if center_yy > 125 and center_yy<316:
                    cv2.circle(frame,(center_xx,center_yy),5,(255,255,255),-1)
                    label = f"{class_names[int(class_label)]}:{confidence:.2f}"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
               
            elapsed_time = time.time() - start_time
            fps = 1 / elapsed_time
            cv2.putText(frame, "FPS: %f" % (fps), (int(20), int(40)), 0, 5e-3 * 200, (0, 255, 0), 3)
            self.signal.emit(frame)

    def load_model(self):
        model = YOLO(r"D:\DATN_ai_yolo\weights\best_can_plastic_bottle_final.pt")
        model.to("cuda")
        print(f"Using device: {model.device}")
        return model

    def load_tracker(self):
        tracker = DeepSort(max_age=30,max_iou_distance= 0.8,n_init= 5,max_cosine_distance= 0.1)
        return tracker

    def stop(self):
        print("stop threading", self.index)
        self.player.release()
        self.running = False
        self.terminate()

    # Nhan du lieu tu ESP32
    def handle_esp32_data_cam(self, data):
        self.esp32_data = data
    
    def reset(self):
        self.tracks = []
        self.detects = []
        self.counter = []
        self.esp32_data = None
        self.mangdem = []
        self.old_count = []
        self.running = False
        self.running = True        

if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_win = tesst_gui()
    main_win.show()
    sys.exit(app.exec())
