#!/usr/bin/env python3
import socket
import struct
import threading
import tkinter as tk

CAN_FRAME = "=IB3x8s"

class Dashboard:
    def __init__(self, root):
        self.root = root
        root.title("Virtual ECU Dashboard")

        self.values = {}
        self.labels = {}

        for name in ["RPM", "Speed", "Throttle", "Brake", "Temperature",
                     "Gear", "Safety"]:
            frame = tk.Frame(root)
            frame.pack(fill="x", padx=10, pady=3)

            tk.Label(frame, text=name, width=15, anchor="w").pack(side="left")
            label = tk.Label(frame, text="-", width=20, anchor="w")
            label.pack(side="left")
            self.labels[name] = label

        self.running = True
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind(("vcan0",))

        threading.Thread(target=self.rx_loop, daemon=True).start()
        root.protocol("WM_DELETE_WINDOW", self.close)

    def rx_loop(self):
        while self.running:
            try:
                raw = self.sock.recv(16)
                can_id, dlc, data = struct.unpack(CAN_FRAME, raw)
                payload = data[:dlc]

                if can_id == 0x100 and len(payload) >= 7:
                    rpm = payload[0] | (payload[1] << 8)
                    speed = (payload[2] | (payload[3] << 8)) * 0.1
                    throttle = payload[4]
                    temp = payload[5]

                    self.values["RPM"] = f"{rpm} rpm"
                    self.values["Speed"] = f"{speed:.1f} km/h"
                    self.values["Throttle"] = f"{throttle} %"
                    self.values["Temperature"] = f"{temp} C"

                elif can_id == 0x110 and len(payload) >= 1:
                    self.values["Brake"] = f"{payload[0]} %"

                elif can_id == 0x120 and len(payload) >= 5:
                    self.values["Gear"] = str(payload[4])

                self.root.after(0, self.refresh)

            except OSError:
                break

    def refresh(self):
        for key, value in self.values.items():
            self.labels[key].config(text=value)

    def close(self):
        self.running = False
        self.sock.close()
        self.root.destroy()

root = tk.Tk()
Dashboard(root)
root.mainloop()