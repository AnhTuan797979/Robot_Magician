# CHỈNH SỬA THÊM PHẦN GỬI NHIỀU TỌA ĐỘ CÙNG LÚC
import tkinter as tk
from tkinter import font
import numpy as np
from PIL import Image, ImageTk
import serial
import time
from tkinter import *
import threading
from PIL import Image, ImageTk
from threading import Thread
from tkinter import messagebox
import cv2
from math import sqrt
from ultralytics import YOLO


# def Doigiaodien()
#     top.destroy() # tắt màn hình đầu
#     Giaodien.deiconify()
x_axis = 15
y_axis = 30
y_lable_IK = 250
ThetaX=0
ThetaY=0
ThetaZ=0
X=0
Y=0
Z=0
x=0
# Px=0; Py=0; Pz=0
l1=150 # 
l2=200 # đã đo 
l3=200 # đã đo ĐO LẠI CÁC LINK
L4=50  # đo nhưng chưa chắc
background_color = "#99FFFF"
#ser = serial.Serial('COM70',115200) #115200
time.sleep(1)
# Khởi tạo cổng nối tiếp (thay đổi COM và baudrate cho phù hợp)
def action(): #ham chuyen doi 2 giao dien                                                                                                                                                                                                                                
    Giaodien.destroy() # tat man hinh dau
    MhDkhien.deiconify() #hien thi man hinh giao dien

    
# Tạo cửa sổ chính

MhDkhien = tk.Tk()
MhDkhien.withdraw() # ẩn màn hình chính
Giaodien = tk.Toplevel()
Giaodien.geometry('1309x647')
Giaodien.title("Điều khiển Robot 3 Bậc Tự Do")
hinh = Image.open("Robotgiaodien.png") # Tải ảnh lên
hinhgd = ImageTk.PhotoImage(hinh) # chuyển đổi hình ảnh
label = tk.Label(Giaodien, image=hinhgd) # hiển thị hình ảnh
label.pack()
Doimanhinh = tk.Button(Giaodien, text="Control Screen", bg="green", fg ="white", width=15,
                       activebackground="Red", command=action,font=("Arial", 15)) # cài đặt nút bấm
Doimanhinh.place(x=600,y=600) # vị trí nút bấm

# Vòng lặp chính để giữ cửa sổ mở
MhDkhien.title("Control Screen")
MhDkhien.configure(bg=background_color) # nền của màn hình ĐK, mã Hex
MhDkhien.geometry('1280x550')

# Tạo Canvas để vẽ
canvas = tk.Canvas(MhDkhien, width=800, height=600, bg=background_color)
canvas.pack(fill=tk.BOTH, expand=True)
canvas.create_line(700, 0, 700, 600, fill="black", width=4)

#------------------------ CÁC HÀM XỬ LÝ --------------------------------#
def DonghocThuan(value1,value2,value3):
    ThetaX = np.radians(value1)
    ThetaY = np.radians(value2)
    ThetaZ = np.radians(value3)
    X = round(np.cos(ThetaX)*(l3*np.cos(ThetaY + ThetaZ) + l2*np.sin(ThetaY))+np.cos(ThetaX)*L4, 2)
    Y = round(np.sin(ThetaX)*(l3*np.cos(ThetaY + ThetaZ) + l2*np.sin(ThetaY))+np.sin(ThetaX)*L4,2)
    Z = round(l1 - l3*np.sin(ThetaY + ThetaZ) + l2*np.cos(ThetaY),2)
    return X,Y,Z

def DonghocNghich(Px,Py,Pz):
    ThetaX_temp = round(np.degrees(np.arctan2(Py, Px)), 2)
    Px_temp = round(Px - np.cos(np.radians(ThetaX_temp)) * L4, 2)
    Py_temp = round(Py - np.sin(np.radians(ThetaX_temp)) * L4, 2)
    Pz_temp = round(Pz, 2)
    ThetaX = round(np.degrees(np.arctan2(Py_temp, Px_temp)), 2)
    term1 = (-Px_temp**2 - Py_temp**2 - Pz_temp**2 + 2 * Pz_temp * l1 - l1**2 + l2**2 + 2 * l2 * l3 + l3**2)
    term2 = (Px_temp**2 + Py_temp**2 + Pz_temp**2 - 2 * Pz_temp * l1 + l1**2 - l2**2 + 2 * l2 * l3 - l3**2)
    term3 = np.sqrt(term1 * term2) if term1 * term2 >= 0 else np.nan
    q2 = -2 * np.arctan((term3 - 2 * l2 * np.sqrt(Px_temp**2 + Py_temp**2)) / 
                        (Px_temp**2 + Py_temp**2 + Pz_temp**2 - 2 * Pz_temp * l1 + 2 * Pz_temp * l2 + l1**2 - 2 * l1 * l2 + l2**2 - l3**2))
    ThetaY = round(np.degrees(q2), 2)
    numerator_q3 = 2 * l2 * l3 - term3
    denominator_q3 = (Px_temp**2 + Py_temp**2 + Pz_temp**2 - 2 * Pz_temp * l1 + l1**2 - l2**2 - l3**2)
    if numerator_q3 == 0:
        q3 = 0
    elif denominator_q3 == 0:
        q3 = np.nan
    else:
        q3 = -2 * np.arctan(numerator_q3 / denominator_q3)
    ThetaZ = round(np.degrees(q3), 2) if not np.isnan(q3) else np.nan
    return ThetaX, ThetaY, ThetaZ

def slider_theta1_value():
    txb_slider_theta1.delete(0,END)    
    ThetaX = slider_theta1.get() # Xóa dữ liệu trước đó
    # txb_slider_theta1.delete(0,END)
    txb_slider_theta1.insert(0,ThetaX) # hiển thị dữ liệu mới
    return ThetaX

def slider_theta2_value():
    txb_slider_theta2.delete(0,END) 
    ThetaY = slider_theta2.get()
    # txb_slider_theta2.delete(0,END)   
    txb_slider_theta2.insert(0,ThetaY)
    return ThetaY

def slider_theta3_value():
    txb_slider_theta3.delete(0,END) 
    ThetaZ = slider_theta3.get()
    # txb_slider_theta3.delete(0,END)
    txb_slider_theta3.insert(0,ThetaZ)  
    return ThetaZ    

def Forwark_Kinematic_btn():
    # cập nhật dữ liệu qua lại từ thanhh slider và textBox theta
    slider_theta1.set(txb_slider_theta1.get())
    slider_theta2.set(txb_slider_theta2.get())
    slider_theta3.set(txb_slider_theta3.get())

    txb_Px_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Py_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Pz_FK.delete(0,END) # Xóa dữ liệu trước đó

    ThetaX = float(txb_slider_theta1.get())
    ThetaY = float(txb_slider_theta2.get())
    ThetaZ = float(txb_slider_theta3.get())

    Px = DonghocThuan(ThetaX,ThetaY,ThetaZ)[0]
    Py = DonghocThuan(ThetaX,ThetaY,ThetaZ)[1]
    Pz = DonghocThuan(ThetaX,ThetaY,ThetaZ)[2]
    txb_Px_FK.insert(0,Px) # hiển thị dữ liệu mới
    txb_Py_FK.insert(0,Py) # hiển thị dữ liệu mới
    txb_Pz_FK.insert(0,Pz) # hiển thị dữ liệu mới
    mang = f"F{ThetaX}A{ThetaY}B{ThetaZ}C"
    print(mang,type(mang))
    ser.write(mang.encode())
    time.sleep(0.01)
    print(ThetaX, ThetaY, ThetaZ)

def FK(x):
    txb_Px_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Py_FK.delete(0,END) # Xóa dữ liệu trước đó
    txb_Pz_FK.delete(0,END) # Xóa dữ liệu trước đó
    ThetaX = slider_theta1_value()
    ThetaY = slider_theta2_value()
    ThetaZ = slider_theta3_value()
    Px = DonghocThuan(ThetaX,ThetaY,ThetaZ)[0]
    Py = DonghocThuan(ThetaX,ThetaY,ThetaZ)[1]
    Pz = DonghocThuan(ThetaX,ThetaY,ThetaZ)[2]
    txb_Px_FK.insert(0,Px) # hiển thị dữ liệu mới vao PX PY PZ
    txb_Py_FK.insert(0,Py) # hiển thị dữ liệu mới
    txb_Pz_FK.insert(0,Pz) # hiển thị dữ liệu mới

def IK():
    Entry_ThetaX_Inv.delete(0,tk.END)
    Entry_ThetaY_Inv.delete(0,tk.END)
    Entry_ThetaZ_Inv.delete(0,tk.END)
    Px = float(Entry_X_Inv.get())
    Py = float(Entry_Y_Inv.get())
    Pz = float(Entry_Z_Inv.get())
    theta1 = DonghocNghich(Px,Py,Pz)[0]
    theta2 = DonghocNghich(Px,Py,Pz)[1]
    theta3 = DonghocNghich(Px,Py,Pz)[2]
    Entry_ThetaX_Inv.insert(0,theta1)
    Entry_ThetaY_Inv.insert(0,theta2)
    Entry_ThetaZ_Inv.insert(0,theta3)
    mang = f'I{theta1}A{theta2}B{theta3}C'
    print(mang)
    ser.write(mang.encode()) 
    time.sleep(0.01)  #để thêm độ trễ trong quá trình thực thi chương trình. 

def Reset_Slider():
    txb_slider_theta1.delete(0,END)
    txb_slider_theta2.delete(0,END)
    txb_slider_theta3.delete(0,END) 
    txb_slider_theta1.insert(0,ThetaX)
    txb_slider_theta2.insert(0,ThetaY)
    txb_slider_theta3.insert(0,ThetaZ)    
    slider_theta1.set('0') #đặt lại vị trí thanh slider tương ứng với giá trị lấy từ textbox
    slider_theta2.set('0')
    slider_theta3.set('0')

def Reset_lable_Slider_FK():
    Reset_Slider()
    Px = 0
    Py = 0
    Pz = 0 
    txb_Px_FK.delete(0,END) # hiển thị dữ liệu mới
    txb_Py_FK.delete(0,END) # hiển thị dữ liệu mới
    txb_Pz_FK.delete(0,END) # hiển thị dữ liệu mới 
    txb_Px_FK.insert(0,Px) # hiển thị dữ liệu mới
    txb_Py_FK.insert(0,Py) # hiển thị dữ liệu mới
    txb_Pz_FK.insert(0,Pz) # hiển thị dữ liệu mới    

def Reset_IK():
    Px = 250
    Py = 0
    Pz = 350 
    Entry_X_Inv.delete(0,END) # hiển thị dữ liệu mới
    Entry_Y_Inv.delete(0,END) # hiển thị dữ liệu mới
    Entry_Z_Inv.delete(0,END) # hiển thị dữ liệu mới 
    Entry_X_Inv.insert(250,Px) # hiển thị dữ liệu mới
    Entry_Y_Inv.insert(0,Py) # hiển thị dữ liệu mới
    Entry_Z_Inv.insert(350,Pz) # hiển thị dữ liệu mới    
 
def ReSet_FK():
    new_thread = threading.Thread(target=Reset_lable_Slider_FK) # Thread là các hàm hay thủ tục chạy độc lập đối với chương trình chính
    new_thread.start()

def ReSet_IK():
    new_thread = threading.Thread(target=Reset_IK) # Thread là các hàm hay thủ tục chạy độc lập đối với chương trình chính
    new_thread.start()

def SetHome_btn():
    mang = 'S'
    print(mang,type(mang))
    ser.write(mang.encode())
    time.sleep(0.01)

def hut_btn():
    mang = 'H'
    print(mang,type(mang))
    ser.write(mang.encode())
    time.sleep(0.01)
    
def tha_btn():
    mang = 'T'
    print(mang,type(mang))
    ser.write(mang.encode())
    time.sleep(0.01)

#------------------------ END CÁC HÀM XỬ LÝ --------------------------------#

#----------------CÁC HÀM DÙNG CHO XỬ LÝ ẢNH-------------------------------------
# Biến toàn cục để dừng camera
running = False
# Biến toàn cục để lưu tọa độ
x_th0, y_th0, x_th1, y_th1, x_th2, y_th2 = 0, 0, 0, 0, 0, 0

# Tải mô hình YOLOv8
model = YOLO('best_5.pt')  # Đảm bảo bạn đã tải mô hình của mình

# Thiết lập tọa độ vùng quan tâm (ROI)
roi = (20, 20, 620, 470)  # Vùng quan tâm (x1, y1, x2, y2)

# Thiết lập ngưỡng màu HSV cho Red, Green
color_ranges = {
    "Red": [(0, 100, 100), (10, 255, 255)],
    "Green": [(36, 50, 70), (89, 255, 255)],
}

# Màu sắc để vẽ bounding box và chấm tâm tương ứng
drawing_colors = {
    "Red": (0, 0, 255),
    "Green": (0, 255, 0),
}

label_colors = {
    0: (123, 107, 35),  # Màu đỏ cho Label 0
    1: (255, 0, 255),  # Màu xanh lá cho Label 1
    2: (255, 255, 0) # Màu vàng cho Label 2
}

label_texts = {
    0: "Circle",
    1: "Square",
    2: "Triangle",
}

# Hàm tính khoảng cách giữa hai tâm
def calculate_distances(objects):
    distances = []
    red_centers = [
        obj["center"] for obj in objects if obj["color"] == "Red"]
    green_centers = [
        obj["center"] for obj in objects if obj["color"] == "Green"]

    for red_center in red_centers:
        for green_center in green_centers:
            distance = sqrt(
                (green_center[0] - red_center[0]) ** 2
                + (green_center[1] - red_center[1]) ** 2
            )
            distances.append((red_center, green_center, distance))

    return distances

# Hàm vẽ khung ROI
def draw_static_frame(frame, roi):
    x1, y1, x2, y2 = roi
    cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
    return frame

# Hàm xử lý ảnh bằng YOLO để phát hiện vật thể
def detect_objects(frame):
    frame = draw_static_frame(frame, roi)
    x1_roi, y1_roi, x2_roi, y2_roi = roi

    # Phát hiện vật thể bằng YOLO
    results = model.predict(source=frame, conf=0.5, show=False)
    coordinates = {"label_1": [], "label_2": [], "label_3": []}

    for box in results[0].boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        label = int(box.cls.item())
        confidence = float(box.conf.item())
        



        if x1_roi <= x1 <= x2_roi and y1_roi <= y1 <= y2_roi:
            if label == 0:
                coordinates["label_1"].append((x1, y1, x2, y2, label, confidence))
            elif label == 1:
                coordinates["label_2"].append((x1, y1, x2, y2, label, confidence))
            elif label == 2:
                coordinates["label_3"].append((x1, y1, x2, y2, label, confidence))

            # Lấy màu dựa trên nhãn (label), mặc định là màu trắng nếu không tìm thấy nhãn
            box_color = label_colors.get(label, (255, 255, 255)) 
            label_text = label_texts.get(label, "Unknown")  # Mặc định là "Unknown" 
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)  # Vẽ box với màu tương ứng

            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            cv2.circle(frame, (center_x, center_y), 5, box_color, -1)  # Vẽ chấm cùng màu với box

            # Hiển thị text (nhãn và độ tin cậy) lên hình ảnh
            cv2.putText(
                frame,
                f"{label_text} ({confidence:.2f})",  # Text: Tên nhãn và độ tin cậy
                (x1, y1 - 10),  # Hiển thị ở phía trên bên trái bounding box
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,  # Kích thước chữ
                box_color,  # Màu chữ
                2,  # Độ dày chữ
                cv2.LINE_AA  # Loại đường vẽ
            )

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    detected_objects = []

    for color_name, (lower, upper) in color_ranges.items():
        lower_bound = np.array(lower, dtype=np.uint8)
        upper_bound = np.array(upper, dtype=np.uint8)
        mask = cv2.inRange(hsv, lower_bound, upper_bound)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:
                x, y, w, h = cv2.boundingRect(contour)
                center_x, center_y = x + w // 2, y + h // 2

                if x1_roi <= center_x <= x2_roi and y1_roi <= center_y <= y2_roi:
                    detected_objects.append({
                        "color": color_name,
                        "area": area,
                        "box": (x, y, x + w, y + h),
                        "center": (center_x, center_y)
                    })
                    box_color = drawing_colors[color_name]
                    cv2.rectangle(frame, (x, y), (x + w, y + h), box_color, 2)
                    cv2.circle(frame, (center_x, center_y), 5, box_color, -1)

 

    return frame, coordinates, detected_objects

# Hàm mở camera và hiển thị thông tin trong cửa sổ riêng
def open_camera():
    global running , x_th0, y_th0, x_th1, y_th1, x_th2, y_th2 
    running = True
    cap = cv2.VideoCapture(1)

    def update_frame():
        if not running:
            cap.release()
            cv2.destroyAllWindows()
            return

        ret, frame = cap.read()
        if ret:
            frame, coordinates, detected_objects = detect_objects(frame)
            distances = calculate_distances(detected_objects)

            txb_detect_XLA.delete("1.0", tk.END)
            txb_detect_Tron.delete("1.0", tk.END)
            txb_detect_vuong.delete("1.0", tk.END)
            txb_detect_TamGiac.delete("1.0", tk.END)

            if distances:
                txb_detect_XLA.insert(tk.END, f"Distance: {distances[0][2]:.2f}")
            else:
                txb_detect_XLA.insert(tk.END, "Distance: N/A\n")

            # Hiển thị thông tin tọa độ cho từng nhãn (Label 1, Label 2, Label 3)
            for coord in coordinates["label_1"]:
                x1, y1, x2, y2, label, confidence = coord
                # Tính toán tọa độ tâm của vật label 1
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                # Tính tọa độ tương đối so với tâm vật đỏ (0,0)
                relative_x = center_x - distances[0][0][0]
                relative_y = center_y - distances[0][0][1]
            
            # Tính toán x_th0, y_th0 cho label 0
                global x_th0, y_th0
                if distances[0][2] != 0:
                    x_th0 = 20 / distances[0][2] * relative_y + 200
                    y_th0 = 20 / distances[0][2] * relative_x 
                else:
                    x_th0 = y_th0 = 0  # Nếu distance == 0 thì không tính toán được

                txb_detect_Tron.insert(tk.END, f"x_th0: {x_th0:.2f}, y_th0: {y_th0:.2f}")

            for coord in coordinates["label_2"]:
                x1, y1, x2, y2, label, confidence = coord
                # Tính toán tọa độ tâm của vật label 2
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                # Tính tọa độ tương đối so với tâm vật đỏ (0,0)
                relative_x = center_x - distances[0][0][0]
                relative_y = center_y - distances[0][0][1]

                # Tính toán x_th1, y_th1 cho label 1
                global x_th1, y_th1
                if distances[0][2] != 0:
                        x_th1 = 20 / distances[0][2] * relative_y + 200
                        y_th1 = 20 / distances[0][2] * relative_x 
                else:
                    x_th1 = y_th1 = 0  # Nếu distance == 0 thì không tính toán được

                txb_detect_vuong.insert(tk.END, f"x_th1: {x_th1:.2f}, y_th1: {y_th1:.2f}")

            for coord in coordinates["label_3"]:
                x1, y1, x2, y2, label, confidence = coord
                # Tính toán tọa độ tâm của vật label 3
                center_x = (x1 + x2) // 2
                center_y = (y1 + y2) // 2
                # Tính tọa độ tương đối so với tâm vật đỏ (0,0)
                relative_x = center_x - distances[0][0][0]
                relative_y = center_y - distances[0][0][1]

                # Tính toán x_th2, y_th2 cho label 2
                global x_th2, y_th2
                if distances[0][2] != 0:
                    x_th2 = 20 / distances[0][2] * relative_y + 200
                    y_th2 = 20 / distances[0][2] * relative_x 
                else:
                    x_th2 = y_th2 = 0  # Nếu distance == 0 thì không tính toán được

                txb_detect_TamGiac.insert(tk.END, f"x_th2: {x_th2:.2f}, y_th2: {y_th2:.2f}")

            cv2.imshow("Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                close_camera()

        MhDkhien.after(10, update_frame)

    update_frame()

# Hàm dừng camera
def close_camera():
    global running
    running = False
    cv2.destroyAllWindows()

# Hàm xử lý khi nhấn nút
def update_coordinates():
    global x_th0, y_th0, x_th1, y_th1, x_th2, y_th2
    # Đảm bảo các biến có giá trị
    txb_theta1_PP_1.delete("1.0", tk.END)
    txb_theta1_PP_2.delete("1.0", tk.END)
    txb_theta1_PP_3.delete("1.0", tk.END)
    txb_theta2_PP_1.delete("1.0", tk.END)
    txb_theta2_PP_2.delete("1.0", tk.END)
    txb_theta2_PP_3.delete("1.0", tk.END)
    txb_theta3_PP_1.delete("1.0", tk.END)
    txb_theta3_PP_2.delete("1.0", tk.END)
    txb_theta3_PP_3.delete("1.0", tk.END)

    txb_theta1_PP_1.insert(tk.END, f"{x_th0:.2f}")
    txb_theta1_PP_2.insert(tk.END, f"{y_th0:.2f}")
    txb_theta1_PP_3.insert(tk.END, 42)
    txb_theta2_PP_1.insert(tk.END, f"{x_th1:.2f}")
    txb_theta2_PP_2.insert(tk.END, f"{y_th1:.2f}")
    txb_theta2_PP_3.insert(tk.END, 46)
    txb_theta3_PP_1.insert(tk.END, f"{x_th2:.2f}")
    txb_theta3_PP_2.insert(tk.END, f"{y_th2:.2f}")
    txb_theta3_PP_3.insert(tk.END, 48)


def Run_XLA_btn():
        # Sử dụng chỉ số "1.0" và tk.END để lấy nội dung từ các widget Text
    AX = float(txb_theta1_PP_1.get("1.0", tk.END).strip()) if txb_theta1_PP_1.get("1.0", tk.END).strip() else 0.0
    AY = float(txb_theta1_PP_2.get("1.0", tk.END).strip()) if txb_theta1_PP_2.get("1.0", tk.END).strip() else 0.0
    AZ = float(txb_theta1_PP_3.get("1.0", tk.END).strip()) if txb_theta1_PP_3.get("1.0", tk.END).strip() else 0.0

    BX = float(txb_theta2_PP_1.get("1.0", tk.END).strip()) if txb_theta2_PP_1.get("1.0", tk.END).strip() else 0.0
    BY = float(txb_theta2_PP_2.get("1.0", tk.END).strip()) if txb_theta2_PP_2.get("1.0", tk.END).strip() else 0.0
    BZ = float(txb_theta2_PP_3.get("1.0", tk.END).strip()) if txb_theta2_PP_3.get("1.0", tk.END).strip() else 0.0

    CX = float(txb_theta3_PP_1.get("1.0", tk.END).strip()) if txb_theta3_PP_1.get("1.0", tk.END).strip() else 0.0
    CY = float(txb_theta3_PP_2.get("1.0", tk.END).strip()) if txb_theta3_PP_2.get("1.0", tk.END).strip() else 0.0
    CZ = float(txb_theta3_PP_3.get("1.0", tk.END).strip()) if txb_theta3_PP_3.get("1.0", tk.END).strip() else 0.0

    ThetaAX = DonghocNghich(AX, AY, AZ)[0]
    ThetaAY = DonghocNghich(AX, AY, AZ)[1]
    ThetaAZ = DonghocNghich(AX, AY, AZ)[2]

    ThetaBX = DonghocNghich(BX, BY, BZ)[0]
    ThetaBY = DonghocNghich(BX, BY, BZ)[1]
    ThetaBZ = DonghocNghich(BX, BY, BZ)[2]

    ThetaCX = DonghocNghich(CX, CY, CZ)[0]
    ThetaCY = DonghocNghich(CX, CY, CZ)[1]
    ThetaCZ = DonghocNghich(CX, CY, CZ)[2]

    mang = f"N{ThetaAX}A{ThetaAY}B{ThetaAZ}C{ThetaBX}D{ThetaBY}E{ThetaBZ}F{ThetaCX}G{ThetaCY}H{ThetaCZ}T"
    ser.write(str(mang).encode())  # Gửi chuỗi qua Serial
    print(mang, type(mang))
    time.sleep(0.01)

#----------------END CÁC HÀM DÙNG CHO XỬ LÝ ẢNH-------------------------------------



#------------------------------ CÁC LABEL CHO PHẦNG ĐỘNG HỌC --------------------------------------------
TenManhinh=tk.Label(MhDkhien,text="CONTROL SCREEN",fg="red",bd=2,font=("Arial",20,font.BOLD),bg=background_color)
TenManhinh.place(x=120 , y=0)
#-------------------ĐỘNG HỌC THUẬN--------------------------#
            #-----Nhập 3 gốc theta
lbl_FK = tk.Label(MhDkhien,text="FORWARD KINEMATIC",fg="black",font=("Arial",14,font.BOLD),bg=background_color)
lbl_FK.place(x=x_axis,y=y_axis)
#-----------3 label Theta-----------------------------------------------------------------------
Label_ThetaX = tk.Label(MhDkhien,text="ThetaX",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
Label_ThetaX.place(x=x_axis,y=y_axis+40) # Vị trí để Label (X hàng ngang, Y hàng dọc)

Label_ThetaY = tk.Label(MhDkhien,text="ThetaY",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
Label_ThetaY.place(x=15,y=y_axis+85)

Label_ThetaZ = tk.Label(MhDkhien,text="ThetaZ",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
Label_ThetaZ.place(x=15,y=y_axis+130)

#------------ Thanh trượt-------------------------------------------------------------
slider_theta1 = tk.Scale(MhDkhien,from_=-90, to_= 90,orient="horizontal",width=15,resolution=0.2,length=350,command=FK) # gioi han thanh slider
slider_theta1.set(ThetaX) # cài đặt vị trí ban đầu cho thanh trượt
slider_theta1.place(x=x_axis+60,y=y_axis+30)
txb_slider_theta1 = tk.Entry(MhDkhien,width=6,font=("Arial",12,font.BOLD)) #tao o hien thi goc ben canh slider
txb_slider_theta1.insert(0,ThetaX)
txb_slider_theta1.place(x=x_axis+420,y=y_axis+40)


#------------------------Thanh Trượt Y------------------------------
slider_theta2 = tk.Scale(MhDkhien,from_=-40, to_= 90,orient="horizontal",width=15,resolution=0.2,length=350,command=FK)
slider_theta2.set(ThetaY)
slider_theta2.place(x=x_axis+60,y=y_axis+75)
txb_slider_theta2 = tk.Entry(MhDkhien,width=6,font=("Arial",12,font.BOLD))
txb_slider_theta2.insert(0,ThetaY)
txb_slider_theta2.place(x=x_axis+420,y=y_axis+80)
btn_Set_Theta2 = tk.Button(MhDkhien,text="Forwark Kinematic",font=("Arial",12,font.BOLD),width=15,height=2,bg='#98FB98',command=Forwark_Kinematic_btn)
btn_Set_Theta2.place(x=x_axis+480,y=y_axis+35)

#------------------------Thanh Trượt Z------------------------------
slider_theta3 = tk.Scale(MhDkhien,from_=-20, to_= 90,orient="horizontal",width=15,resolution=0.2,length=350,command=FK)
slider_theta3.set(ThetaZ)
slider_theta3.place(x=x_axis+60,y=y_axis+120)
txb_slider_theta3 = tk.Entry(MhDkhien,width=6,font=("Arial",12,font.BOLD))
txb_slider_theta3.insert(0,ThetaZ)
txb_slider_theta3.place(x=x_axis+420,y=y_axis+130)

#--------------------Xuất giá trị X,Y,Z-----------------------------------------------------
Label_X = tk.Label(MhDkhien, text="X",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
Label_X.place(x=x_axis+100,y=y_axis+170)
txb_Px_FK = tk.Entry(MhDkhien,width=7,font=("Arial",12,font.BOLD))
txb_Px_FK.place(x=x_axis+80,y=y_axis+203)

Label_Y = tk.Label(MhDkhien, text="Y",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
Label_Y.place(x=x_axis+200,y=y_axis+170)
txb_Py_FK = tk.Entry(MhDkhien,width=7,font=("Arial",12,font.BOLD))
txb_Py_FK.place(x=x_axis+180,y=y_axis+203)

Label_Z = tk.Label(MhDkhien, text="Z",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
Label_Z.place(x=x_axis+300,y=y_axis+170)
txb_Pz_FK = tk.Entry(MhDkhien,width=7,font=("Arial",12,font.BOLD))
txb_Pz_FK.place(x=x_axis+280,y=y_axis+203)

#--------------------ĐỘNG HỌC NGHỊCH----------------------#
#------Xuất giá trị THETA X,Y,Z
lbl_IK = tk.Label(MhDkhien,text="INVERSE KINEMATIC",fg="black",font=("Arial",14,font.BOLD),bg=background_color)
lbl_IK.place(x=x_axis,y=y_axis+y_lable_IK)

Label_ThetaX_Inv = tk.Label(MhDkhien, text="ThetaX=",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
Label_ThetaX_Inv.place(x=200,y=y_axis+y_lable_IK+53) # Vị trí để Label (X hàng ngang, Y hàng dọc)
Entry_ThetaX_Inv = tk.Entry(MhDkhien,width=9,font=("Arial",12,font.BOLD)) # Tạo ô nhập dữ liệu
Entry_ThetaX_Inv.place(x=270,y=y_axis+y_lable_IK+53)

Label_ThetaY_Inv = tk.Label(MhDkhien, text="ThetaY=",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
Label_ThetaY_Inv.place(x=200,y=y_axis+y_lable_IK+103)
Entry_ThetaY_Inv = tk.Entry(MhDkhien,width=9,font=("Arial",12,font.BOLD))
Entry_ThetaY_Inv.place(x=270,y=y_axis+y_lable_IK+103)

Label_ThetaZ_Inv = tk.Label(MhDkhien, text="ThetaZ=",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
Label_ThetaZ_Inv.place(x=200,y=y_axis+y_lable_IK+153)
Entry_ThetaZ_Inv = tk.Entry(MhDkhien,width=9,font=("Arial",12,font.BOLD))
Entry_ThetaZ_Inv.place(x=270,y=y_axis+y_lable_IK+153)
#---------Nhập giá trị X,Y,Z
Label_X_Inv = tk.Label(MhDkhien, text="X=",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
Label_X_Inv.place(x=x_axis+20,y=y_axis+y_lable_IK+50)
Entry_X_Inv = tk.Entry(MhDkhien,width=10,font=("Arial",12,font.BOLD))
Entry_X_Inv.place(x=x_axis+50,y=y_axis+y_lable_IK+53)
Label_Y_Inv = tk.Label(MhDkhien, text="Y=",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
Label_Y_Inv.place(x=x_axis+20,y=y_axis+y_lable_IK+100)
Entry_Y_Inv = tk.Entry(MhDkhien,width=10,font=("Arial",12,font.BOLD))
Entry_Y_Inv.place(x=x_axis+50,y=y_axis+y_lable_IK+103)
Label_Z_Inv = tk.Label(MhDkhien, text="Z=",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
Label_Z_Inv.place(x=x_axis+20,y=y_axis+y_lable_IK+150)
Entry_Z_Inv = tk.Entry(MhDkhien,width=10,font=("Arial",12,font.BOLD))
Entry_Z_Inv.place(x=x_axis+50,y=y_axis+y_lable_IK+153)

#------------------Các nút--------------------#
btn_Start = tk.Button(MhDkhien,text="Set Home",font=("Arial",12,font.BOLD),width=10,height=2,bg='#00CC33',command=SetHome_btn)
btn_Start.place(x=x_axis+500,y=y_axis+240)

btn_IV = Button(MhDkhien,text="Invert Kinematic",font=("Arial",12,font.BOLD),width=13,height=2,bg='#98FB98',command=IK)
#command=IK
btn_IV.place(x=x_axis+40,y=y_axis+y_lable_IK+190)
btn_ReSet_FK = tk.Button(MhDkhien,text="Reset Forward",font=("Arial",12,font.BOLD),width=12,height=2,bg='#FF9933',command=ReSet_FK)
btn_ReSet_FK.place(x=x_axis+480,y=y_axis+100)

btn_ReSet_IV = tk.Button(MhDkhien,text="Reset Inverse",font=("Arial",12,font.BOLD),width=13,height=2,bg='#FF9933',command=ReSet_IK)
btn_ReSet_IV.place(x=x_axis+210,y=y_axis+y_lable_IK+190)

btn_hut = tk.Button(MhDkhien,text="Hut",font=("Arial",10,font.BOLD),width=8,height=2,bg='#98FB98',command=hut_btn)
btn_hut.place(x=x_axis+400,y=y_axis+360)
btn_tha = tk.Button(MhDkhien,text="Tha",font=("Arial",10,font.BOLD),width=8,height=2,bg='#98FB98',command=tha_btn)
btn_tha.place(x=x_axis+490,y=y_axis+360)
#----------------------------------------------------------------------------------------------------------------





#--------------------------  XỬ LÝ ẢNH  -------------------------------#
TenManhinh_1=tk.Label(MhDkhien,text="XỬ LÝ ẢNH",fg="red",bd=2,font=("Arial",20,font.BOLD),bg=background_color)
TenManhinh_1.place(x=900 , y=0)


lbl_theta_PP_1 = Label(MhDkhien,text="Px",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta_PP_2 = Label(MhDkhien,text="Py",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta_PP_3 = Label(MhDkhien,text="Pz",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_theta_PP_1.place(x=885,y=y_axis+25)
lbl_theta_PP_2.place(x=1035,y=y_axis+25)
lbl_theta_PP_3.place(x=1185,y=y_axis+25)

lbl_LINK_PB_1 = Label(MhDkhien,text="TRÒN",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_LINK_PB_2 = Label(MhDkhien,text="VUÔNG",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_LINK_PB_3 = Label(MhDkhien,text="TAM GIÁC",fg="black",font=("Arial",12,font.BOLD),bg=background_color)
lbl_LINK_PB_1.place(x=750,y=y_axis+55)
lbl_LINK_PB_2.place(x=750,y=y_axis+105)
lbl_LINK_PB_3.place(x=750,y=y_axis+155)


# txb_theta1_PP_1 = Entry(MhDkhien,width=10,font=("Arial",12,font.BOLD)) 
# txb_theta2_PP_1 = Entry(MhDkhien,width=10,font=("Arial",12,font.BOLD))
# txb_theta3_PP_1 = Entry(MhDkhien,width=10,font=("Arial",12,font.BOLD))
# txb_theta1_PP_2 = Entry(MhDkhien,width=10,font=("Arial",12,font.BOLD)) 
# txb_theta2_PP_2 = Entry(MhDkhien,width=10,font=("Arial",12,font.BOLD))
# txb_theta3_PP_2 = Entry(MhDkhien,width=10,font=("Arial",12,font.BOLD))
# txb_theta1_PP_3 = Entry(MhDkhien,width=10,font=("Arial",12,font.BOLD)) 
# txb_theta2_PP_3 = Entry(MhDkhien,width=10,font=("Arial",12,font.BOLD))
# txb_theta3_PP_3 = Entry(MhDkhien,width=10,font=("Arial",12,font.BOLD))

txb_theta1_PP_1 = tk.Text(MhDkhien, height=2,width=10) 
txb_theta2_PP_1 = tk.Text(MhDkhien, height=2,width=10)
txb_theta3_PP_1 = tk.Text(MhDkhien, height=2,width=10)
txb_theta1_PP_2 = tk.Text(MhDkhien, height=2,width=10) 
txb_theta2_PP_2 = tk.Text(MhDkhien, height=2,width=10)
txb_theta3_PP_2 = tk.Text(MhDkhien, height=2,width=10)
txb_theta1_PP_3 = tk.Text(MhDkhien, height=2,width=10) 
txb_theta2_PP_3 = tk.Text(MhDkhien, height=2,width=10)
txb_theta3_PP_3 = tk.Text(MhDkhien, height=2,width=10)

txb_theta1_PP_1.place(x=850,y=y_axis+53)
txb_theta2_PP_1.place(x=850,y=y_axis+103)
txb_theta3_PP_1.place(x=850,y=y_axis+153)
txb_theta1_PP_2.place(x=1000,y=y_axis+53)
txb_theta2_PP_2.place(x=1000,y=y_axis+103)
txb_theta3_PP_2.place(x=1000,y=y_axis+153)
txb_theta1_PP_3.place(x=1150,y=y_axis+53)
txb_theta2_PP_3.place(x=1150,y=y_axis+103)
txb_theta3_PP_3.place(x=1150,y=y_axis+153)

# TextBox hiển thị thông tin của tọa độ của 3 vật 
txb_detect_XLA = Text(MhDkhien,height=2, width=55,font=("Arial",12,font.BOLD)) 
txb_detect_XLA.place(x=750 , y=300)

txb_detect_Tron = Text(MhDkhien,height=2, width=55,font=("Arial",12,font.BOLD)) 
txb_detect_Tron.place(x=750 , y=350)

txb_detect_vuong = Text(MhDkhien,height=2, width=55,font=("Arial",12,font.BOLD)) 
txb_detect_vuong.place(x=750 , y=400)

txb_detect_TamGiac = Text(MhDkhien,height=2, width=55,font=("Arial",12,font.BOLD)) 
txb_detect_TamGiac.place(x=750 , y=450)


# Nút "Open Camera"
open_camera_btn = Button(MhDkhien, text="Open Camera",font=("Arial",10,font.BOLD),width=12,height=2,bg='#98FB98', command=open_camera)
open_camera_btn.place(x=800 ,y=230)


# Nút "Close Camera"
close_camera_btn = Button(MhDkhien, text="Close Camera",font=("Arial",10,font.BOLD),width=12,height=2,bg='#98FB98', command=close_camera)
close_camera_btn.place(x=930 ,y=230)

# Nút "SET POINT" chốt tọa độ của vật
close_camera_btn = Button(MhDkhien, text="UPDATE DATA",font=("Arial",10,font.BOLD),width=10,height=2,bg='#98FB98', command=update_coordinates)
close_camera_btn.place(x=1060 ,y=230)
# Nút "Run XLA" chạy phân loại vật  
btn_SETPOINT = Button(MhDkhien,text="Run XLA",font=("Arial",10,font.BOLD),width=10,height=2,bg='#98FB98',command=Run_XLA_btn)
btn_SETPOINT.place(x=1180 ,y=230)


#---------------------------------------END XỬ LÝ ẢNH -------------------------#



MhDkhien.mainloop()

