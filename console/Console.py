import tkinter as tk
from tkinter import Canvas
from PIL import Image, ImageTk
import cv2

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
        self.robot_direction = 0  
        self.robot_on_map = self.map_canvas.create_image(self.robot_x, self.robot_y, image=self.robot_photo, anchor=tk.CENTER)
        
        self.coordinates_label = tk.Label(self.root, text=f"Current: ({self.robot_x}, {self.robot_y})", font=("Arial", 12))
        self.coordinates_label.place(x=500, y=520)
        
        self.target_label = tk.Label(self.root, text="Target: (x, y)", font=("Arial", 12))
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
