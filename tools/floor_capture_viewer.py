"""Native Tk viewer; works with both GUI and headless OpenCV installations."""
from collections import deque
import tkinter as tk

import cv2


class CaptureViewer:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("床校正の撮影 — C:保存 / A・B:基準点反転 / Q:終了")
        self.root.protocol("WM_DELETE_WINDOW", self._request_close)
        self.closed = False
        self.keys = deque()
        self.photo = None
        self.image = tk.Label(self.root)
        self.image.pack()
        tk.Label(self.root, text="両画面の黄色いREFを同じ印付き基準点に合わせてください。 C / Space：保存　A：aux反転　B：base反転　Q / Esc：終了").pack()
        self.root.bind("<KeyPress>", self._on_key)
        self.root.update()

    def _request_close(self):
        self.closed = True

    def _on_key(self, event):
        if event.keysym == "Escape":
            self.keys.append(27)
        elif event.keysym == "space":
            self.keys.append(ord(" "))
        elif event.char and event.char.lower() in ("a", "b", "c", "q"):
            self.keys.append(ord(event.char.lower()))

    def show(self, canvas):
        if self.closed:
            return
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        height, width = rgb.shape[:2]
        ppm = f"P6\n{width} {height}\n255\n".encode("ascii") + rgb.tobytes()
        self.photo = tk.PhotoImage(data=ppm, format="PPM", master=self.root)
        self.image.configure(image=self.photo)

    def poll_key(self):
        if self.closed:
            return 27
        self.root.update_idletasks()
        self.root.update()
        if self.closed:
            return 27
        return self.keys.popleft() if self.keys else -1

    def close(self):
        try:
            self.root.destroy()
        except tk.TclError:
            pass
        self.closed = True
