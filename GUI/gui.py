

import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import math
import time
from collections import deque

# ── Theme ───────────────────────────────────────────────────
BG      = "#0a0e1a"
PANEL   = "#111827"
BORDER  = "#1e293b"
ACCENT  = "#00e5ff"
ACCENT2 = "#7c3aed"
SUCCESS = "#10b981"
WARNING = "#f59e0b"
DANGER  = "#ef4444"
TEXT    = "#f1f5f9"
DIM     = "#64748b"
GRID    = "#1e293b"
FONT    = "Consolas"
HISTORY = 300


class ArmController:
    def __init__(self, root):
        self.root = root
        self.root.title("ROBOT ARM CONTROLLER")
        self.root.configure(bg=BG)
        self.root.geometry("1400x900")
        self.root.minsize(1200, 750)
        try:
            self.root.tk.call("tk", "scaling", 1.3)
        except:
            pass

        # Serial
        self.ser       = None
        self.connected = False

        # Live data
        self.actual_angle  = 0.0
        self.target_angle  = 0.0
        self.error_val     = 0.0
        self.p_term        = 0.0
        self.i_term        = 0.0
        self.d_term        = 0.0
        self.pwm_val       = 0.0
        self.display_angle = 0.0   # smoothed for animation

        # History — graph 1: system response
        self.hist_target = deque([0.0]*HISTORY, maxlen=HISTORY)
        self.hist_actual = deque([0.0]*HISTORY, maxlen=HISTORY)
        self.hist_error  = deque([0.0]*HISTORY, maxlen=HISTORY)

        # History — graph 2: PID internals
        self.hist_p   = deque([0.0]*HISTORY, maxlen=HISTORY)
        self.hist_i   = deque([0.0]*HISTORY, maxlen=HISTORY)
        self.hist_d   = deque([0.0]*HISTORY, maxlen=HISTORY)
        self.hist_pwm = deque([0.0]*HISTORY, maxlen=HISTORY)

        # Tkinter vars
        self.kp_var     = tk.StringVar(value="1.2")
        self.ki_var     = tk.StringVar(value="0.0")
        self.kd_var     = tk.StringVar(value="0.35")
        self.port_var   = tk.StringVar(value="COM11")
        self.status_var = tk.StringVar(value="DISCONNECTED")

        self._build_ui()
        self._refresh()

    # ── BUILD UI ──────────────────────────────────────────
    def _build_ui(self):
        self._build_topbar()

        content = tk.Frame(self.root, bg=BG)
        content.pack(fill=tk.BOTH, expand=True, padx=12, pady=(0, 12))

        # Left column
        left = tk.Frame(content, bg=BG, width=310)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        left.pack_propagate(False)
        self._build_gauge(left)
        self._build_controls(left)
        self._build_pid(left)

        # Right column
        right = tk.Frame(content, bg=BG)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self._build_arm_visual(right)
        self._build_graph1(right)   # System response
        self._build_graph2(right)   # PID internals
        self._build_log(right)

    # ── TOP BAR ───────────────────────────────────────────
    def _build_topbar(self):
        bar = tk.Frame(self.root, bg=PANEL, height=52)
        bar.pack(fill=tk.X)
        bar.pack_propagate(False)

        tk.Label(bar, text="⬡  ROBOT ARM CONTROLLER",
                 bg=PANEL, fg=ACCENT,
                 font=(FONT, 14, "bold")).pack(side=tk.LEFT, padx=20)

        self.status_dot = tk.Label(bar, text="●", bg=PANEL,
                                   fg=DANGER, font=("Arial", 16))
        self.status_dot.pack(side=tk.LEFT, padx=(10, 4))
        self.status_lbl = tk.Label(bar, textvariable=self.status_var,
                                   bg=PANEL, fg=DANGER, font=(FONT, 9, "bold"))
        self.status_lbl.pack(side=tk.LEFT)

        self.btn_connect = tk.Button(bar, text="CONNECT",
                                     bg=ACCENT2, fg=TEXT,
                                     font=(FONT, 9, "bold"),
                                     relief=tk.FLAT, padx=14, pady=5,
                                     cursor="hand2",
                                     command=self._toggle_connect)
        self.btn_connect.pack(side=tk.RIGHT, padx=12, pady=10)

        self.port_combo = ttk.Combobox(bar, textvariable=self.port_var,
                                       width=9, font=(FONT, 9))
        self.port_combo.pack(side=tk.RIGHT, padx=4, pady=14)
        self._refresh_ports()

        tk.Label(bar, text="PORT:", bg=PANEL, fg=DIM,
                 font=(FONT, 8)).pack(side=tk.RIGHT, padx=4)
        tk.Button(bar, text="⟳", bg=PANEL, fg=DIM,
                  font=("Arial", 13), relief=tk.FLAT, cursor="hand2",
                  command=self._refresh_ports).pack(side=tk.RIGHT, padx=4)

    # ── GAUGE ─────────────────────────────────────────────
    def _build_gauge(self, parent):
        card = self._card(parent, "ENCODER  360°")
        self.gauge_cv = tk.Canvas(card, width=290, height=290,
                                  bg=PANEL, highlightthickness=0)
        self.gauge_cv.pack(pady=6)
        row = tk.Frame(card, bg=PANEL)
        row.pack(fill=tk.X, padx=14, pady=(0, 10))
        self.lbl_actual = tk.Label(row, text="0.0°", bg=PANEL, fg=ACCENT,
                                   font=(FONT, 24, "bold"))
        self.lbl_actual.pack(side=tk.LEFT)
        tk.Label(row, text="ACTUAL", bg=PANEL, fg=DIM,
                 font=(FONT, 8)).pack(side=tk.LEFT, padx=6, pady=6)
        self.lbl_target2 = tk.Label(row, text="→ 0.0°", bg=PANEL, fg=ACCENT2,
                                    font=(FONT, 12))
        self.lbl_target2.pack(side=tk.RIGHT)

    def _draw_gauge(self, actual, target):
        c = self.gauge_cv
        c.delete("all")
        cx = cy = 145
        r = 118
        c.create_oval(cx-r, cy-r, cx+r, cy+r, outline=BORDER, fill="#0d1420", width=2)
        for d in range(0, 360, 5):
            a = math.radians(d - 90)
            inner = r - (10 if d % 30 == 0 else 5)
            c.create_line(cx+inner*math.cos(a), cy+inner*math.sin(a),
                          cx+r*math.cos(a),     cy+r*math.sin(a),
                          fill=DIM if d % 30 == 0 else BORDER)
        for d in range(0, 360, 30):
            a = math.radians(d - 90)
            c.create_text(cx+(r-22)*math.cos(a), cy+(r-22)*math.sin(a),
                          text=str(d), fill=DIM, font=(FONT, 7))
        # Target
        tn = target % 360
        if tn:
            c.create_arc(cx-r+10, cy-r+10, cx+r-10, cy+r-10,
                         start=90, extent=-tn, outline=ACCENT2, style=tk.ARC, width=4)
        ta = math.radians(tn - 90)
        c.create_line(cx, cy, cx+(r-16)*math.cos(ta), cy+(r-16)*math.sin(ta),
                      fill=ACCENT2, width=2, dash=(5, 3))
        # Actual
        an = actual % 360
        c.create_arc(cx-r+18, cy-r+18, cx+r-18, cy+r-18,
                     start=90, extent=-an, outline=ACCENT, style=tk.ARC, width=6)
        aa = math.radians(an - 90)
        c.create_line(cx, cy, cx+(r-14)*math.cos(aa), cy+(r-14)*math.sin(aa),
                      fill=ACCENT, width=3)
        c.create_oval(cx-5, cy-5, cx+5, cy+5, fill=ACCENT, outline="")
        err = target - actual
        c.create_text(cx, cy-12, text=f"{actual:.1f}°",
                      fill=ACCENT, font=(FONT, 13, "bold"))
        c.create_text(cx, cy+10,
                      text=f"ERR {err:+.1f}°",
                      fill=WARNING if abs(err) > 2 else SUCCESS,
                      font=(FONT, 9))

    # ── CONTROLS ──────────────────────────────────────────
    def _build_controls(self, parent):
        card = self._card(parent, "MOTION  CONTROL")

        # Custom angle
        tk.Label(card, text="CUSTOM ANGLE (deg)",
                 bg=PANEL, fg=DIM, font=(FONT, 8)).pack(anchor=tk.W, padx=14, pady=(6, 2))
        row = tk.Frame(card, bg=PANEL)
        row.pack(fill=tk.X, padx=14, pady=(0, 6))
        self.custom_entry = tk.Entry(row, bg=BORDER, fg=ACCENT,
                                     font=(FONT, 15, "bold"), width=7,
                                     justify=tk.CENTER, insertbackground=ACCENT,
                                     bd=0, highlightthickness=1,
                                     highlightcolor=ACCENT, highlightbackground=BORDER)
        self.custom_entry.insert(0, "0")
        self.custom_entry.pack(side=tk.LEFT, ipady=7, padx=(0, 6))
        self.custom_entry.bind("<Return>", lambda e: self._send_custom())
        tk.Button(row, text="GO  ▶", bg=ACCENT, fg=BG,
                  font=(FONT, 10, "bold"), relief=tk.FLAT, padx=14, pady=7,
                  cursor="hand2", command=self._send_custom).pack(side=tk.LEFT, fill=tk.X, expand=True)

        # Increment buttons
        tk.Label(card, text="INCREMENT / DECREMENT",
                 bg=PANEL, fg=DIM, font=(FONT, 8)).pack(anchor=tk.W, padx=14, pady=(4, 2))
        inc = tk.Frame(card, bg=PANEL)
        inc.pack(fill=tk.X, padx=14, pady=(0, 6))
        for lbl, d in [("−45",-45),("−10",-10),("−1",-1),("+1",1),("+10",10),("+45",45)]:
            tk.Button(inc, text=lbl,
                      bg=DANGER if d < 0 else SUCCESS, fg=BG,
                      font=(FONT, 8, "bold"), relief=tk.FLAT,
                      padx=5, pady=6, cursor="hand2",
                      command=lambda v=d: self._increment(v)
                      ).pack(side=tk.LEFT, padx=1)

        # Presets
        tk.Label(card, text="QUICK PRESETS",
                 bg=PANEL, fg=DIM, font=(FONT, 8)).pack(anchor=tk.W, padx=14, pady=(4, 2))
        pre = tk.Frame(card, bg=PANEL)
        pre.pack(fill=tk.X, padx=14, pady=(0, 8))
        for a in [-90, -45, 0, 45, 90]:
            tk.Button(pre, text=f"{a}°", bg=BORDER, fg=ACCENT,
                      font=(FONT, 9, "bold"), relief=tk.FLAT,
                      padx=7, pady=5, cursor="hand2",
                      command=lambda v=a: self._send_target(v)
                      ).pack(side=tk.LEFT, padx=2)

        # Zero + Stop
        act = tk.Frame(card, bg=PANEL)
        act.pack(fill=tk.X, padx=14, pady=(0, 6))
        tk.Button(act, text="⊙ ZERO HERE", bg=WARNING, fg=BG,
                  font=(FONT, 9, "bold"), relief=tk.FLAT, padx=12, pady=7,
                  cursor="hand2", command=self._send_zero
                  ).pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))
        tk.Button(act, text="⏹ STOP", bg=DANGER, fg=TEXT,
                  font=(FONT, 9, "bold"), relief=tk.FLAT, padx=12, pady=7,
                  cursor="hand2", command=self._send_stop
                  ).pack(side=tk.LEFT)

        self.lbl_cur = tk.Label(card, text="CURRENT TARGET:  0.0°",
                                bg=PANEL, fg=DIM, font=(FONT, 9))
        self.lbl_cur.pack(pady=(0, 8))

    # ── PID PANEL ─────────────────────────────────────────
    def _build_pid(self, parent):
        card = self._card(parent, "PID  TUNING")
        for label, var, prefix, color in [
            ("Kp  PROPORTIONAL", self.kp_var, "kp", ACCENT),
            ("Ki  INTEGRAL",     self.ki_var, "ki", SUCCESS),
            ("Kd  DERIVATIVE",   self.kd_var, "kd", WARNING),
        ]:
            row = tk.Frame(card, bg=PANEL)
            row.pack(fill=tk.X, padx=14, pady=3)
            tk.Label(row, text=label, bg=PANEL, fg=color,
                     font=(FONT, 8), width=19, anchor=tk.W).pack(side=tk.LEFT)
            tk.Entry(row, textvariable=var, bg=BORDER, fg=color,
                     font=(FONT, 11, "bold"), width=7, justify=tk.CENTER,
                     insertbackground=color, bd=0,
                     highlightthickness=1, highlightcolor=color,
                     highlightbackground=BORDER
                     ).pack(side=tk.LEFT, padx=5, ipady=4)
            tk.Button(row, text="SET", bg=BORDER, fg=color,
                      font=(FONT, 8, "bold"), relief=tk.FLAT,
                      padx=8, pady=4, cursor="hand2",
                      command=lambda p=prefix, v=var: self._send_pid(p, v)
                      ).pack(side=tk.LEFT)

        tk.Button(card, text="SEND ALL PID", bg=ACCENT2, fg=TEXT,
                  font=(FONT, 9, "bold"), relief=tk.FLAT,
                  padx=12, pady=7, cursor="hand2",
                  command=self._send_all_pid
                  ).pack(fill=tk.X, padx=14, pady=(6, 12))

    # ── ARM VISUAL ────────────────────────────────────────
    def _build_arm_visual(self, parent):
        card = self._card(parent, "ARM  VISUALIZATION")
        card.pack(side=tk.TOP, fill=tk.X, pady=(0, 6))
        self.arm_cv = tk.Canvas(card, height=170, bg=PANEL, highlightthickness=0)
        self.arm_cv.pack(fill=tk.X, padx=14, pady=6)

    def _draw_arm(self, actual, target):
        c = self.arm_cv
        c.delete("all")
        w = c.winfo_width() or 700
        h = 170
        cx, cy, L = w//2, h-25, 120
        c.create_rectangle(0, cy+20, w, h, fill="#0d1420", outline="")
        c.create_line(0, cy+20, w, cy+20, fill=BORDER)
        c.create_rectangle(cx-22, cy, cx+22, cy+20, fill=BORDER, outline=DIM)
        c.create_rectangle(cx-32, cy+18, cx+32, cy+28, fill=DIM, outline="")
        # Target ghost
        ta = math.radians(90 - target)
        tx, ty = cx+L*math.cos(ta), cy-L*math.sin(ta)
        c.create_line(cx, cy, tx, ty, fill=ACCENT2, width=2, dash=(6, 4))
        c.create_oval(tx-7, ty-7, tx+7, ty+7, fill="", outline=ACCENT2, width=2)
        c.create_text(tx+16, ty, text=f"TARGET\n{target:.1f}°",
                      fill=ACCENT2, font=(FONT, 8), justify=tk.CENTER)
        # Actual arm
        aa = math.radians(90 - actual)
        ax, ay = cx+L*math.cos(aa), cy-L*math.sin(aa)
        c.create_line(cx+3, cy+3, ax+3, ay+3, fill="#000", width=12)
        c.create_line(cx, cy, ax, ay, fill=ACCENT, width=9, capstyle=tk.ROUND)
        if abs(actual) > 1:
            c.create_arc(cx-50, cy-50, cx+50, cy+50,
                         start=90, extent=-actual,
                         outline=ACCENT, style=tk.ARC, width=2)
        c.create_oval(cx-11, cy-11, cx+11, cy+11, fill=DIM, outline=ACCENT, width=2)
        c.create_oval(ax-10, ay-10, ax+10, ay+10, fill=ACCENT, outline=TEXT, width=2)
        c.create_text(ax+16, ay-10, text=f"ACTUAL\n{actual:.1f}°",
                      fill=ACCENT, font=(FONT, 8, "bold"), justify=tk.CENTER)

    # ── GRAPH 1 — System Response ──────────────────────────
    def _build_graph1(self, parent):
        card = self._card(parent, "GRAPH 1 — SYSTEM RESPONSE  (Target / Actual / Error)")
        card.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=(0, 4))
        self.g1_cv = tk.Canvas(card, bg=PANEL, highlightthickness=0)
        self.g1_cv.pack(fill=tk.BOTH, expand=True, padx=14, pady=4)
        leg = tk.Frame(card, bg=PANEL)
        leg.pack(pady=(0, 4))
        for color, label in [(ACCENT2,"TARGET"),(ACCENT,"ACTUAL"),(WARNING,"ERROR")]:
            tk.Label(leg, text="━━", bg=PANEL, fg=color,
                     font=("Arial", 11)).pack(side=tk.LEFT, padx=3)
            tk.Label(leg, text=label, bg=PANEL, fg=DIM,
                     font=(FONT, 8)).pack(side=tk.LEFT, padx=(0, 10))

    # ── GRAPH 2 — PID Internals ────────────────────────────
    def _build_graph2(self, parent):
        card = self._card(parent, "GRAPH 2 — PID INTERNALS  (P / I / D / PWM)")
        card.pack(side=tk.TOP, fill=tk.BOTH, expand=True, pady=(0, 4))
        self.g2_cv = tk.Canvas(card, bg=PANEL, highlightthickness=0)
        self.g2_cv.pack(fill=tk.BOTH, expand=True, padx=14, pady=4)
        leg = tk.Frame(card, bg=PANEL)
        leg.pack(pady=(0, 4))
        for color, label in [(ACCENT,"P-TERM"),(SUCCESS,"I-TERM"),(WARNING,"D-TERM"),(DANGER,"PWM")]:
            tk.Label(leg, text="━━", bg=PANEL, fg=color,
                     font=("Arial", 11)).pack(side=tk.LEFT, padx=3)
            tk.Label(leg, text=label, bg=PANEL, fg=DIM,
                     font=(FONT, 8)).pack(side=tk.LEFT, padx=(0, 10))

    def _draw_graph(self, canvas, series_list, title=""):
        c = canvas
        c.delete("all")
        w = c.winfo_width() or 600
        h = c.winfo_height() or 100
        if w < 20 or h < 20:
            return
        PL, PR, PT, PB = 44, 10, 6, 18
        cw, ch = w-PL-PR, h-PT-PB
        c.create_rectangle(PL, PT, w-PR, h-PB, fill="#0d1420", outline=BORDER)

        # Y range across all series
        all_v = [v for hist, _, _ in series_list for v in hist]
        if not all_v:
            return
        ymin, ymax = min(all_v)-10, max(all_v)+10
        if (ymax-ymin) < 10:
            mid=(ymax+ymin)/2; ymin=mid-10; ymax=mid+10

        # Grid + Y labels
        for i in range(5):
            yg = PT + i*ch//4
            c.create_line(PL, yg, w-PR, yg, fill=GRID, dash=(2, 4))
            val = ymin + (ymax-ymin)*(4-i)/4
            c.create_text(PL-4, yg, text=f"{val:.0f}",
                          fill=DIM, font=(FONT, 7), anchor=tk.E)

        def tx(i): return PL + int(i*cw/(HISTORY-1))
        def ty(v): return PT + ch - int((v-ymin)/(ymax-ymin)*ch)

        for hist, color, width in series_list:
            pts = list(hist)
            coords = []
            for i, v in enumerate(pts):
                coords += [tx(i), ty(v)]
            if len(coords) >= 4:
                c.create_line(*coords, fill=color, width=width, smooth=True)

        # Live dot for first series
        if series_list and series_list[0][0]:
            lx = tx(HISTORY-1)
            ly = ty(list(series_list[0][0])[-1])
            c.create_oval(lx-3, ly-3, lx+3, ly+3,
                          fill=series_list[0][1], outline="")

    # ── LOG ───────────────────────────────────────────────
    def _build_log(self, parent):
        card = self._card(parent, "SERIAL  LOG")
        card.pack(side=tk.TOP, fill=tk.X)
        self.log_txt = tk.Text(card, height=4, bg="#0d1420", fg=SUCCESS,
                               font=(FONT, 8), relief=tk.FLAT,
                               state=tk.DISABLED)
        self.log_txt.pack(fill=tk.X, padx=14, pady=6)

    # ── HELPER ────────────────────────────────────────────
    def _card(self, parent, title):
        f = tk.Frame(parent, bg=PANEL, highlightthickness=1,
                     highlightbackground=BORDER)
        f.pack(fill=tk.X, pady=(0, 6))
        tk.Label(f, text=f"  {title}  ", bg=BORDER, fg=ACCENT,
                 font=(FONT, 8, "bold"), anchor=tk.W).pack(fill=tk.X)
        return f

    # ── SERIAL ────────────────────────────────────────────
    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if "COM11" in ports:
            self.port_var.set("COM11")
        elif ports:
            self.port_var.set(ports[0])

    def _toggle_connect(self):
        self._disconnect() if self.connected else self._connect()

    def _connect(self):
        try:
            self.ser = serial.Serial(self.port_var.get(), 115200, timeout=0.1)
            self.connected = True
            self.status_var.set(f"CONNECTED  {self.port_var.get()}")
            self.status_dot.config(fg=SUCCESS)
            self.status_lbl.config(fg=SUCCESS)
            self.btn_connect.config(text="DISCONNECT", bg=DANGER)
            self._log(f"Connected to {self.port_var.get()}")
            threading.Thread(target=self._read_serial, daemon=True).start()
        except Exception as e:
            self.status_var.set("FAILED")
            self._log(f"Error: {e}")

    def _disconnect(self):
        self.connected = False
        if self.ser:
            self.ser.close()
        self.status_var.set("DISCONNECTED")
        self.status_dot.config(fg=DANGER)
        self.status_lbl.config(fg=DANGER)
        self.btn_connect.config(text="CONNECT", bg=ACCENT2)
        self._log("Disconnected")

    def _read_serial(self):
        while self.connected:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        self._parse_line(line)
            except:
                break
            time.sleep(0.002)

    def _parse_line(self, line):
        # Expected: Angle:45.2 Target:90.0 Err:44.8 P:53.0 I:2.1 D:-8.0 PWM:120
        try:
            if "Angle:" in line:
                parts = line.replace(":", " ").split()
                # Index map: Angle[0] val[1] Target[2] val[3] Err[4] val[5]
                #            P[6] val[7] I[8] val[9] D[10] val[11] PWM[12] val[13]
                self.actual_angle = float(parts[1])
                self.target_angle = float(parts[3])
                self.error_val    = float(parts[5])

                self.hist_actual.append(self.actual_angle)
                self.hist_target.append(self.target_angle)
                self.hist_error.append(self.error_val)

                # PID terms — only if ESP32 sends them
                if len(parts) >= 14:
                    self.p_term  = float(parts[7])
                    self.i_term  = float(parts[9])
                    self.d_term  = float(parts[11])
                    self.pwm_val = float(parts[13])
                else:
                    # Estimate from available data if ESP32 doesn't send terms
                    try:
                        kp = float(self.kp_var.get())
                        ki = float(self.ki_var.get())
                        kd = float(self.kd_var.get())
                        self.p_term = kp * self.error_val
                    except:
                        pass

                self.hist_p.append(self.p_term)
                self.hist_i.append(self.i_term)
                self.hist_d.append(self.d_term)
                self.hist_pwm.append(self.pwm_val)
            else:
                self._log(line)
        except:
            self._log(line)

    def _send(self, cmd):
        if self.connected and self.ser:
            try:
                self.ser.write((cmd+"\n").encode())
                self._log(f"→ {cmd}")
            except Exception as e:
                self._log(f"Send error: {e}")
        else:
            self._log(f"[NOT CONNECTED] → {cmd}")

    def _log(self, msg):
        def _do():
            self.log_txt.config(state=tk.NORMAL)
            self.log_txt.insert(tk.END, f"  {msg}\n")
            self.log_txt.see(tk.END)
            self.log_txt.config(state=tk.DISABLED)
        self.root.after(0, _do)

    # ── COMMANDS ──────────────────────────────────────────
    def _send_custom(self):
        try:
            self._send_target(float(self.custom_entry.get()))
        except ValueError:
            self._log("Invalid angle")

    def _send_target(self, angle):
        self.target_angle = float(angle)
        self.hist_target.append(self.target_angle)
        self.lbl_cur.config(text=f"CURRENT TARGET:  {angle:.1f}°")
        self.custom_entry.delete(0, tk.END)
        self.custom_entry.insert(0, str(angle))
        self._send(f"a{angle:.1f}")

    def _increment(self, delta):
        try:
            current = float(self.custom_entry.get())
        except:
            current = self.target_angle
        self._send_target(round(current + delta, 1))

    def _send_zero(self):
        self._send("z")
        self.target_angle = 0.0
        self.custom_entry.delete(0, tk.END)
        self.custom_entry.insert(0, "0")

    def _send_stop(self):
        self._send("s")

    def _send_pid(self, prefix, var):
        self._send(f"{prefix}{var.get()}")

    def _send_all_pid(self):
        for prefix, var in [("kp", self.kp_var),
                             ("ki", self.ki_var),
                             ("kd", self.kd_var)]:
            self._send(f"{prefix}{var.get()}")
            time.sleep(0.05)

    # ── REFRESH 60 FPS ────────────────────────────────────
    def _refresh(self):
        actual = self.actual_angle
        target = self.target_angle

        # Smooth animation
        self.display_angle += (actual - self.display_angle) * 0.25

        self._draw_gauge(self.display_angle, target)
        self.lbl_actual.config(text=f"{actual:.1f}°")
        self.lbl_target2.config(text=f"→ {target:.1f}°")
        self._draw_arm(self.display_angle, target)

        # Graph 1 — system response
        self._draw_graph(self.g1_cv, [
            (self.hist_target, ACCENT2, 1.5),
            (self.hist_actual, ACCENT,  2.0),
            (self.hist_error,  WARNING, 1.0),
        ])

        # Graph 2 — PID internals
        self._draw_graph(self.g2_cv, [
            (self.hist_p,   ACCENT,   1.5),
            (self.hist_i,   SUCCESS,  1.5),
            (self.hist_d,   WARNING,  1.5),
            (self.hist_pwm, DANGER,   2.0),
        ])

        self.root.after(16, self._refresh)


# ── MAIN ────────────────────────────────────────────────────
if __name__ == "__main__":
    root = tk.Tk()
    app  = ArmController(root)

    def on_close():
        app._disconnect()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()
