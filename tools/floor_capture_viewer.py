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
        tk.Label(self.root, text="基準点：白黒格子の下端中央（REF BOTTOM）。格子を検出すると、基準点に黄色の丸が出ます。").pack()
        tk.Label(self.root, text="C / Space：保存　A：aux反転　B：base反転　＋/－：画面の明るさ　Q / Esc：終了").pack()
        buttons = tk.Frame(self.root)
        buttons.pack()
        tk.Button(buttons, text="明るく（＋）", command=lambda: self.keys.append(ord("+"))).pack(side=tk.LEFT)
        tk.Button(buttons, text="暗く（－）", command=lambda: self.keys.append(ord("-"))).pack(side=tk.LEFT)
        self.root.bind("<KeyPress>", self._on_key)
        self.root.update()

    def _request_close(self):
        self.closed = True

    def _on_key(self, event):
        if event.keysym == "Escape":
            self.keys.append(27)
        elif event.keysym == "space":
            self.keys.append(ord(" "))
        elif event.keysym in ("plus", "equal", "KP_Add"):
            self.keys.append(ord("+"))
        elif event.keysym in ("minus", "KP_Subtract"):
            self.keys.append(ord("-"))
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
