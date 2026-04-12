#!/usr/bin/env python3
"""Simple GUI for ESP dual-servo control.

ESP command format:
- A <0-180>
- P <1000-2000>
"""

import tkinter as tk
from tkinter import ttk
import threading

import serial
from serial.tools import list_ports


class ServoGui:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("ESP Servo Control")
        self.root.geometry("700x520")

        self.ser: serial.Serial | None = None
        self.read_thread: threading.Thread | None = None
        self.stop_event = threading.Event()

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.angle_var = tk.IntVar(value=90)
        self.pwm_var = tk.IntVar(value=1500)
        self.angle_after_id: str | None = None
        self.pwm_after_id: str | None = None

        self._build_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self) -> None:
        top = ttk.Frame(self.root, padding=10)
        top.pack(fill="x")

        ttk.Label(top, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=22, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=6)

        ttk.Button(top, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=6)

        ttk.Label(top, text="Baud:").grid(row=0, column=3, sticky="e")
        ttk.Entry(top, textvariable=self.baud_var, width=10).grid(row=0, column=4, padx=6)

        self.connect_btn = ttk.Button(top, text="Connect", command=self.connect)
        self.connect_btn.grid(row=0, column=5, padx=6)

        self.disconnect_btn = ttk.Button(top, text="Disconnect", command=self.disconnect, state="disabled")
        self.disconnect_btn.grid(row=0, column=6, padx=6)

        servo_frame = ttk.LabelFrame(self.root, text="Servo Controls", padding=10)
        servo_frame.pack(fill="x", padx=10, pady=8)

        ttk.Label(servo_frame, text="Angle Servo (Pin 25):").grid(row=0, column=0, sticky="w")
        angle_scale = ttk.Scale(
            servo_frame,
            from_=0,
            to=180,
            variable=self.angle_var,
            command=self.on_angle_slider,
        )
        angle_scale.grid(row=0, column=1, padx=8, sticky="ew")
        self.angle_value_label = ttk.Label(servo_frame, text="90", width=5)
        self.angle_value_label.grid(row=0, column=2)

        ttk.Label(servo_frame, text="Continuous Servo PWM (Pin 33):").grid(row=1, column=0, sticky="w", pady=(10, 0))
        pwm_scale = ttk.Scale(
            servo_frame,
            from_=1000,
            to=2000,
            variable=self.pwm_var,
            command=self.on_pwm_slider,
        )
        pwm_scale.grid(row=1, column=1, padx=8, sticky="ew", pady=(10, 0))
        self.pwm_value_label = ttk.Label(servo_frame, text="1500", width=5)
        self.pwm_value_label.grid(row=1, column=2, pady=(10, 0))

        servo_frame.columnconfigure(1, weight=1)

        quick = ttk.Frame(self.root, padding=(10, 2))
        quick.pack(fill="x")
        ttk.Button(quick, text="Center + Stop", command=self.center_stop).pack(side="left")

        log_frame = ttk.LabelFrame(self.root, text="Serial Log", padding=10)
        log_frame.pack(fill="both", expand=True, padx=10, pady=8)

        self.log = tk.Text(log_frame, height=14, wrap="word")
        self.log.pack(fill="both", expand=True)
        self.log.configure(state="disabled")

    def refresh_ports(self) -> None:
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def append_log(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def connect(self) -> None:
        if self.ser and self.ser.is_open:
            return

        port = self.port_var.get().strip()
        if not port:
            self.append_log("Select a serial port first")
            return

        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            self.append_log("Invalid baud rate")
            return

        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
        except serial.SerialException as exc:
            self.append_log(f"Connect failed: {exc}")
            return

        self.stop_event.clear()
        self.read_thread = threading.Thread(target=self.reader_loop, daemon=True)
        self.read_thread.start()

        self.connect_btn.configure(state="disabled")
        self.disconnect_btn.configure(state="normal")
        self.append_log(f"Connected to {port} @ {baud}")

    def disconnect(self) -> None:
        self.stop_event.set()
        if self.ser:
            try:
                self.ser.close()
            except serial.SerialException:
                pass
        self.ser = None
        self.connect_btn.configure(state="normal")
        self.disconnect_btn.configure(state="disabled")
        self.append_log("Disconnected")

    def reader_loop(self) -> None:
        while not self.stop_event.is_set() and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode(errors="replace").strip()
            except serial.SerialException as exc:
                self.root.after(0, lambda: self.append_log(f"RX error: {exc}"))
                self.root.after(0, self.disconnect)
                return

            if line:
                self.root.after(0, lambda msg=line: self.append_log(f"RX: {msg}"))

    def send_line(self, msg: str) -> None:
        if not self.ser or not self.ser.is_open:
            self.append_log("Not connected")
            return

        try:
            self.ser.write((msg + "\n").encode("utf-8", errors="replace"))
            self.ser.flush()
            self.append_log(f"TX: {msg}")
        except serial.SerialException as exc:
            self.append_log(f"TX error: {exc}")
            self.disconnect()

    def send_angle(self) -> None:
        value = max(0, min(180, int(self.angle_var.get())))
        self.angle_var.set(value)
        self.angle_value_label.config(text=str(value))
        self.send_line(f"A {value}")

    def send_pwm(self) -> None:
        value = max(1000, min(2000, int(self.pwm_var.get())))
        self.pwm_var.set(value)
        self.pwm_value_label.config(text=str(value))
        self.send_line(f"P {value}")

    def on_angle_slider(self, _value: str) -> None:
        self.angle_value_label.config(text=str(self.angle_var.get()))
        if self.angle_after_id is not None:
            self.root.after_cancel(self.angle_after_id)
        self.angle_after_id = self.root.after(80, self.send_angle)

    def on_pwm_slider(self, _value: str) -> None:
        self.pwm_value_label.config(text=str(self.pwm_var.get()))
        if self.pwm_after_id is not None:
            self.root.after_cancel(self.pwm_after_id)
        self.pwm_after_id = self.root.after(80, self.send_pwm)

    def center_stop(self) -> None:
        self.angle_var.set(90)
        self.pwm_var.set(1500)
        self.angle_value_label.config(text="90")
        self.pwm_value_label.config(text="1500")
        self.send_line("A 90")
        self.send_line("P 1500")

    def on_close(self) -> None:
        self.disconnect()
        self.root.destroy()


def main() -> int:
    root = tk.Tk()
    ServoGui(root)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
