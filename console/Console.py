import tkinter as tk
from tkinter import Canvas
from PIL import Image, ImageTk
import cv2
import numpy as np
from pyzbar.pyzbar import decode
from urllib.parse import parse_qs

WINDOW_WIDTH = 1200
WINDOW_HEIGHT = 700
MAP_SIZE = 400  
ROBOT_SIZE = 20  
STEP_SIZE = 10   

class RobotApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Navigation App")
        self.root.geometry(f"{WINDOW_WIDTH}x{WINDOW_HEIGHT}")
        
        self.map_canvas = Canvas(self.root, bg="blue", width=MAP_SIZE, height=MAP_SIZE)
        self.map_canvas.place(x=50, y=100, width=MAP_SIZE, height=MAP_SIZE)
         
        self.video_frame = tk.Label(self.root)
        self.video_frame.place(x=500, y=100, width=MAP_SIZE, height=MAP_SIZE)
            
        self.robot_image = Image.open(r"C:\Users\Zaki\Desktop\AU Final Project Console\Test Codes\arrow.png").resize((ROBOT_SIZE, ROBOT_SIZE))
        self.robot_photo = ImageTk.PhotoImage(self.robot_image)
        self.robot_x, self.robot_y = 10, MAP_SIZE - 10 
        self.target_x = 0
        self.target_y = 0
        self.robot_direction = 0  
        self.robot_on_map = self.map_canvas.create_image(self.robot_x, self.robot_y, image=self.robot_photo, anchor=tk.CENTER)
        
        self.coordinates_label = tk.Label(self.root, text=f"Current: ({self.target_x}, {self.robot_y})", font=("Arial", 12))
        self.coordinates_label.place(x=500, y=520)
        
        self.target_label = tk.Label(self.root, text=f"Target: ({self.robot_x}, {self.target_y})", font=("Arial", 12))
        self.target_label.place(x=500, y=550)

        self.root.bind("<Up>", self.move_forward)
        self.root.bind("<Down>", self.move_backward)
        self.root.bind("<Left>", self.turn_left)
        self.root.bind("<Right>", self.turn_right)
        
        self.cap = cv2.VideoCapture(1)
        self.update_video_feed()

    def update_video_feed(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            qr_codes = decode(gray_frame)

            if qr_codes:
                for qr_code in qr_codes:
                    # Extract the data from the QR code
                    qr_data = qr_code.data.decode('utf-8')
                    print("QR Code Data:", qr_data)
                    
                    start_x, start_y, width, height = qr_code.rect

                    cv2.rectangle (frame, (start_x, start_y), (start_x + width, start_y + height), (0, 255, 0), 2)
                    cv2.putText(frame, qr_data, (start_x, start_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    str_x = ""
                    str_y = ""
                    flag = 0
                    for c in qr_data:
                        if (c == '&'):
                            flag = 1
                            continue ;
                        if (flag and ord (c) >= ord ('0') and ord (c) <= ord ('9')):
                            str_y += c
                        elif (ord (c) >= ord ('0') and ord (c) <= ord ('9')):
                            str_x += c

                    parsed_data = parse_qs(qr_data)

                    self.target_x = float(parsed_data['X'][0])
                    self.target_y = float(parsed_data['Y'][0])

                    self.target_label.config(text=f"Current: ({self.target_x}, {self.target_y})")

                    # if len(points) > 4:
                    #     hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
                    #     cv2.polylines(frame, [hull], True, (255, 0, 255), 2)
                    # else:
                    #     cv2.polylines(frame, [np.array(points, dtype=np.int32)], True, (0, 255, 0), 2)


            frame_image = Image.fromarray(frame)
            frame_photo = ImageTk.PhotoImage(frame_image)
            self.video_frame.config(image=frame_photo)
            self.video_frame.image = frame_photo
        
        self.root.after(10, self.update_video_feed)

    def move_forward(self, event):
        dx, dy = self.get_movement_offset()
        self.robot_x += dx
        self.robot_y += dy
        self.update_robot_position()

    def move_backward(self, event):
        dx, dy = self.get_movement_offset()
        self.robot_x -= dx
        self.robot_y -= dy
        self.update_robot_position()

    def turn_left(self, event):
        self.robot_direction = (self.robot_direction - 90) % 360
        self.rotate_robot()

    def turn_right(self, event):
        self.robot_direction = (self.robot_direction + 90) % 360
        self.rotate_robot()

    def get_movement_offset(self):
        if self.robot_direction == 0:
            return 0, -STEP_SIZE
        elif self.robot_direction == 90:
            return STEP_SIZE, 0 
        elif self.robot_direction == 180:
            return 0, STEP_SIZE  
        elif self.robot_direction == 270:
            return -STEP_SIZE, 0  

    def update_robot_position(self):
        self.robot_x = max(0, min(self.robot_x, MAP_SIZE))
        self.robot_y = max(0, min(self.robot_y, MAP_SIZE))
        
        self.map_canvas.coords(self.robot_on_map, self.robot_x, self.robot_y)
        self.coordinates_label.config(text=f"Current: ({self.robot_x}, {self.robot_y})")

    def rotate_robot(self):
        rotated_image = self.robot_image.rotate(-self.robot_direction)
        self.robot_photo = ImageTk.PhotoImage(rotated_image)
        self.map_canvas.itemconfig(self.robot_on_map, image=self.robot_photo)

    def on_closing(self):
        self.cap.release()
        self.root.destroy()

root = tk.Tk()
app = RobotApp(root)
root.protocol("WM_DELETE_WINDOW", app.on_closing)
root.mainloop()
