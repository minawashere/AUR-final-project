import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
import cv2  # For camera integration


class RobotConsoleApp:
    def __init__(self, main_root):
        self.root = main_root
        self.root.title("Robot Console")

        # Initialize robot coordinates
        self.robot_x = 0.0
        self.robot_y = 0.0

        # Initialize widgets to None
        self.coord_label = None
        self.camera_label = None

        self.create_widgets()

        # Camera setup (optional, for testing)
        self.cap = cv2.VideoCapture(0)  # Replace with actual camera source
        self.update_camera_feed()

    def create_widgets(self):
        # Coordinate Display
        self.coord_label = tk.Label(self.root, text=f"Coordinates: X = {self.robot_x:.2f}, Y = {self.robot_y:.2f}",
                                    font=("Arial", 14))
        self.coord_label.pack(pady=10)

        # Control Panel
        control_frame = tk.Frame(self.root)
        control_frame.pack(pady=10)

        tk.Button(control_frame, text="Move Forward", command=self.move_forward).grid(row=0, column=1, padx=5)
        tk.Button(control_frame, text="Turn Left", command=self.turn_left).grid(row=1, column=0, padx=5)
        tk.Button(control_frame, text="Turn Right", command=self.turn_right).grid(row=1, column=2, padx=5)
        tk.Button(control_frame, text="Move Backward", command=self.move_backward).grid(row=2, column=1, padx=5)

        # Camera Feed Display
        self.camera_label = tk.Label(self.root)
        self.camera_label.pack(pady=10)

        # QR Code Button
        tk.Button(self.root, text="Scan QR Code", command=self.scan_qr_code).pack(pady=10)

    @staticmethod
    def move_forward():     # placeholder
        messagebox.showinfo("Action", "Moving Forward")

    @staticmethod
    def move_backward():     # placeholder
        messagebox.showinfo("Action", "Moving Backward")

    @staticmethod
    def turn_left():     # placeholder
        messagebox.showinfo("Action", "Turning Left")

    @staticmethod
    def turn_right():     # placeholder
        messagebox.showinfo("Action", "Turning Right")

    def update_coordinates(self, x, y):
        self.robot_x, self.robot_y = x, y
        self.coord_label.config(text=f"Coordinates: X = {self.robot_x:.2f}, Y = {self.robot_y:.2f}")

    def update_camera_feed(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = ImageTk.PhotoImage(Image.fromarray(frame))
                self.camera_label.configure(image=img)
                self.camera_label.image = img
        self.root.after(50, self.update_camera_feed)

    @staticmethod
    def scan_qr_code():     # placeholder
        messagebox.showinfo("Action", "Scanning QR Code")

    def on_closing(self):
        if self.cap.isOpened():
            self.cap.release()
        self.root.destroy()


if __name__ == "__main__":
    main_window = tk.Tk()
    app = RobotConsoleApp(main_window)
    main_window.protocol("WM_DELETE_WINDOW", app.on_closing)
    main_window.mainloop()
