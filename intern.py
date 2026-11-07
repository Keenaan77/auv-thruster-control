import math
import time
from collections import deque

import pygame
from pymavlink import mavutil


# ===================== KONFIGURASI =====================
class Config:
    PORT = "COM11"
    BAUD = 115200
    SERVO_PIN = 1

    # axis (lihat panel JOYSTICK; Xbox 360: 0/1 = stick kiri, 2/3 = stick kanan)
    AXIS_FWD, INV_FWD = 1, True      # stick kiri atas/bawah  = maju/mundur
    AXIS_YAW, INV_YAW = 2, False     # stick kanan kiri/kanan = yaw
    AXIS_UP,  INV_UP = 3, True       # stick kanan atas/bawah = naik/turun

    # tombol (lihat nomor di panel JOYSTICK, sesuaikan)
    BTN_SOFT_STOP = 0
    BTN_ESTOP = 2
    BTN_ARM = 3
    BTN_PWR_DOWN = 4
    BTN_PWR_UP = 5
    BTN_DISARM = 6

    GAIN = 20.0          # % tenaga awal (KECIL dulu)
    MAX_GAIN = 50.0      # batas aman %
    DEADZONE = 0.15
    RAMP_UP = 600.0      # satuan MANUAL_CONTROL per detik (1000 = penuh)
    RAMP_DOWN = 400.0
    RAMP_REVERSE = 300.0 # laju saat nyeberang netral (kecil = halus)
    SMOOTH_TAU = 0.2     # detik, penghalus stick
    LINK_TIMEOUT = 2.0
    FPS = 30


# ===================== PIXHAWK =====================
class Pixhawk:
    def __init__(self):
        self.pwm = 0
        self.armed = False
        self.mode = "-"
        self.status = "-"
        self.last_hb = 0.0
        self.last_rx = time.time()
        self.params = {}
        print(f"[MAVLink] Connect {Config.PORT} ...")
        self.m = mavutil.mavlink_connection(Config.PORT, baud=Config.BAUD)
        self.m.wait_heartbeat()
        print("[MAVLink] OK, system", self.m.target_system)
        self.m.mav.request_data_stream_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)
        self.check_params()
        self.m.set_mode("MANUAL")

    def read_param(self, name):
        self.m.mav.param_request_read_send(
            self.m.target_system, self.m.target_component, name.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < 2:
            r = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if r and r.param_id.rstrip("\x00") == name:
                return r.param_value
        return None

    def check_params(self):
        want = {f"SERVO{Config.SERVO_PIN}_FUNCTION": 33, "FS_GCS_ENABLE": 2}
        for name, val in want.items():
            v = self.read_param(name)
            self.params[name] = v is not None and abs(v - val) < 0.01
            print(f"[PARAM] {name} = {v} (harus {val})")
            if not self.params[name]:
                print(f"  -> set {name}={val} di QGC lalu reboot Pixhawk")

    @property
    def params_ok(self):
        return all(self.params.values())

    def send_control(self, x, z, r):
        # x,r: -1000..1000 ; z: 0..1000 (500 = netral)
        self.m.mav.manual_control_send(
            self.m.target_system, int(x), 0, int(z), int(r), 0)

    def arm(self, state):
        self.m.mav.command_long_send(
            self.m.target_system, self.m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1 if state else 0, 21196, 0, 0, 0, 0, 0)

    def zero(self, n=10):
        for _ in range(n):
            self.send_control(0, 500, 0)
            time.sleep(0.02)

    def poll(self, now):
        if now - self.last_hb > 1.0:
            self.m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                      mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_hb = now
        for _ in range(80):
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                break
            self.last_rx = now
            t = msg.get_type()
            if t == "SERVO_OUTPUT_RAW":
                self.pwm = getattr(msg, f"servo{Config.SERVO_PIN}_raw", 0)
            elif t == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.mode = self.m.flightmode
            elif t == "STATUSTEXT":
                self.status = msg.text
                print("[Pixhawk]", msg.text)
            elif (t == "COMMAND_ACK" and msg.result != 0
                  and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM):
                print(f"[ACK] arm/disarm ditolak result={msg.result}")

    @property
    def link_ok(self):
        return time.time() - self.last_rx < Config.LINK_TIMEOUT


# ===================== KANAL GERAK (dengan ramp) =====================
class Channel:
    def __init__(self, scale):
        self.scale = scale       # 10 = x,r (gain*10) ; 5 = z (gain*5)
        self.stick = 0.0
        self.cur = 0.0
        self.sm = 0.0

    def zero(self):
        self.cur = self.sm = 0.0

    def update(self, dt, gain, stop):
        tgt = 0.0 if stop else self.stick * gain * self.scale
        self.sm += (tgt - self.sm) * (dt / (Config.SMOOTH_TAU + dt))
        diff = self.sm - self.cur
        if abs(diff) > 0.01:
            if self.cur * self.sm < 0:
                rate = Config.RAMP_REVERSE
            elif abs(self.sm) > abs(self.cur):
                rate = Config.RAMP_UP
            else:
                rate = Config.RAMP_DOWN
            step = rate * dt
            self.cur = self.sm if abs(diff) <= step else self.cur + math.copysign(step, diff)
        return self.cur


def norm_axis(raw, center, inv):
    c = raw - center
    if abs(c) < Config.DEADZONE:
        return 0.0
    v = math.copysign(min(1.0, (abs(c) - Config.DEADZONE) / (1 - Config.DEADZONE)), c)
    return -v if inv else v


# ===================== TAMPILAN =====================
BG, CARD, EDGE = (11, 15, 23), (21, 28, 41), (44, 56, 80)
WHITE, GREY, DIM = (235, 240, 246), (125, 137, 158), (60, 72, 96)
GREEN, RED, CYAN, ORANGE, YELLOW = (70, 210, 130), (240, 80, 80), (80, 215, 255), (255, 150, 50), (255, 208, 0)


def card(s, rect, title, small):
    pygame.draw.rect(s, CARD, rect, border_radius=12)
    pygame.draw.rect(s, EDGE, rect, 2, border_radius=12)
    s.blit(small.render(title, True, GREY), (rect[0] + 14, rect[1] + 10))


def dot(s, x, y, ok, label, small):
    pygame.draw.circle(s, GREEN if ok else RED, (x, y), 7)
    s.blit(small.render(label, True, WHITE), (x + 14, y - 8))


def gauge(s, cx, cy, r, pwm, font):
    lo, hi = 1100, 1900
    pygame.draw.circle(s, (14, 19, 29), (cx, cy), r)
    for p in range(lo, hi + 1, 100):
        a = math.pi * (1 - (p - lo) / (hi - lo))
        major = p in (1100, 1500, 1900)
        l1, l2 = r - (16 if major else 9), r - 2
        pygame.draw.line(s, YELLOW if p == 1500 else GREY,
                         (cx + l1 * math.cos(a), cy - l1 * math.sin(a)),
                         (cx + l2 * math.cos(a), cy - l2 * math.sin(a)), 3 if major else 1)
    pw = max(lo, min(hi, pwm))
    a = math.pi * (1 - (pw - lo) / (hi - lo))
    pygame.draw.line(s, GREEN if pwm >= 1500 else ORANGE, (cx, cy),
                     (cx + (r - 22) * math.cos(a), cy - (r - 22) * math.sin(a)), 5)
    pygame.draw.circle(s, WHITE, (cx, cy), 7)
    t = font.render(f"{int(pwm)} us", True, WHITE)
    s.blit(t, t.get_rect(center=(cx, cy - r // 2 + 10)))


def graph(s, rect, series, small):
    x, y, w, h = rect
    pygame.draw.rect(s, (14, 19, 29), rect)
    pygame.draw.rect(s, EDGE, rect, 1)
    lo, hi = 1100, 1900
    zy = y + h * (1 - (1500 - lo) / (hi - lo))
    pygame.draw.line(s, DIM, (x, zy), (x + w, zy), 1)
    s.blit(small.render("1500", True, GREY), (x + 4, zy - 16))
    for dq, col in series:
        n = len(dq)
        pts = [(x + w * i / (n - 1), y + h * (1 - (max(lo, min(hi, v)) - lo) / (hi - lo)))
               for i, v in enumerate(dq)]
        pygame.draw.lines(s, col, False, pts, 2)


def joy_panel(s, rect, joy, small, font):
    card(s, rect, "JOYSTICK LIVE", small)
    x0, y0, w, h = rect
    if joy is None:
        s.blit(small.render("Joystick tidak terdeteksi (pakai keyboard)", True, RED), (x0 + 14, y0 + 40))
        return
    s.blit(small.render(joy.get_name()[:34], True, GREY), (x0 + 14, y0 + 30))
    n_ax, n_bt = joy.get_numaxes(), joy.get_numbuttons()
    ax = lambda i: joy.get_axis(i) if i < n_ax else 0.0

    r, cy = 48, y0 + 100
    for name, axx, axy, cx in (("KIRI", 0, 1, x0 + 75), ("KANAN", 2, 3, x0 + 215)):
        pygame.draw.circle(s, (14, 19, 29), (cx, cy), r)
        pygame.draw.circle(s, EDGE, (cx, cy), r, 2)
        vx, vy = ax(axx), ax(axy)
        pos = (int(cx + max(-1, min(1, vx)) * (r - 8)), int(cy + max(-1, min(1, vy)) * (r - 8)))
        act = abs(vx) > Config.DEADZONE or abs(vy) > Config.DEADZONE
        pygame.draw.circle(s, GREEN if act else CYAN, pos, 8)
        lab = small.render(name, True, GREY)
        s.blit(lab, lab.get_rect(center=(cx, cy + r + 14)))

    used_ax = {Config.AXIS_FWD: "maju", Config.AXIS_YAW: "yaw", Config.AXIS_UP: "naik"}
    bx = x0 + 300
    s.blit(small.render("AXIS (kuning = dipakai)", True, GREY), (bx, y0 + 30))
    for i in range(min(n_ax, 8)):
        yy = y0 + 52 + i * 18
        v = ax(i)
        s.blit(small.render(f"{i}{':' + used_ax[i] if i in used_ax else ''}", True,
                            YELLOW if i in used_ax else WHITE), (bx, yy))
        pygame.draw.rect(s, (14, 19, 29), (bx + 60, yy + 2, 90, 12))
        wl = int(45 * max(-1, min(1, v)))
        pygame.draw.rect(s, GREEN if v >= 0 else ORANGE, (bx + 105 + min(0, wl), yy + 2, abs(wl), 12))
        s.blit(small.render(f"{v:+.2f}", True, GREY), (bx + 156, yy))

    fn = {Config.BTN_SOFT_STOP: "STOP", Config.BTN_ESTOP: "E-STOP", Config.BTN_ARM: "ARM",
          Config.BTN_DISARM: "DISARM", Config.BTN_PWR_DOWN: "GAIN-", Config.BTN_PWR_UP: "GAIN+"}
    gx, gy = x0 + 520, y0 + 52
    s.blit(small.render("TOMBOL (kuning = ada fungsi)", True, GREY), (gx, y0 + 30))
    pressed_now = []
    for i in range(min(n_bt, 16)):
        rx, ry = gx + (i % 3) * 112, gy + (i // 3) * 24
        pr = joy.get_button(i)
        used = i in fn
        if pr:
            pressed_now.append(i)
        pygame.draw.rect(s, GREEN if pr else ((40, 52, 74) if used else (24, 31, 44)),
                         (rx, ry, 106, 20), border_radius=4)
        pygame.draw.rect(s, YELLOW if used else EDGE, (rx, ry, 106, 20), 1, border_radius=4)
        t = small.render(f"{i} {fn[i]}" if used else str(i), True, (10, 10, 10) if pr else WHITE)
        s.blit(t, t.get_rect(center=(rx + 53, ry + 10)))

    if pressed_now:
        b = pressed_now[-1]
        msg = f"Ditekan: tombol {b}" + (f" = {fn[b]}" if b in fn else " (belum ada fungsi)")
        col = YELLOW if b in fn else GREY
    else:
        msg, col = "Tekan tombol joystick buat lihat nomor & fungsinya", GREY
    s.blit(font.render(msg, True, col), (x0 + 14, y0 + h - 30))


# ===================== MAIN =====================
def main():
    px = Pixhawk()
    pygame.init()
    pygame.joystick.init()
    joy = None
    if pygame.joystick.get_count():
        joy = pygame.joystick.Joystick(0)
        joy.init()
        print("[Joystick]", joy.get_name())
    else:
        print("[Joystick] tidak ada, pakai keyboard")

    centers = {}
    if joy:
        pygame.event.pump()
        time.sleep(0.2)
        pygame.event.pump()
        for a in (Config.AXIS_FWD, Config.AXIS_YAW, Config.AXIS_UP):
            centers[a] = joy.get_axis(a) if a < joy.get_numaxes() else 0.0

    ch_x, ch_r, ch_z = Channel(10), Channel(10), Channel(5)
    state = {"gain": Config.GAIN, "soft": False, "pressed": ""}

    screen = pygame.display.set_mode((920, 860))
    pygame.display.set_caption("Thruster Control - Pixhawk 6C (MANUAL_CONTROL)")
    font = pygame.font.SysFont("Consolas", 20)
    small = pygame.font.SysFont("Consolas", 15)
    big = pygame.font.SysFont("Consolas", 26, bold=True)
    clock = pygame.time.Clock()
    h_out = deque([1500] * 240, maxlen=240)

    def zero_all():
        for c in (ch_x, ch_r, ch_z):
            c.zero()
        state["soft"] = False

    def estop():
        zero_all()
        px.zero()
        px.arm(False)
        print("[E-STOP]")

    def do_arm():
        zero_all()
        px.zero(3)
        px.arm(True)

    def gain(d):
        state["gain"] = max(0.0, min(Config.MAX_GAIN, state["gain"] + d))

    t_prev = time.time()
    running = True
    try:
        while running:
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    running = False
                elif e.type == pygame.KEYDOWN:
                    k = e.key
                    if k == pygame.K_ESCAPE: running = False
                    elif k == pygame.K_RETURN: do_arm()
                    elif k in (pygame.K_BACKSPACE, pygame.K_x): estop()
                    elif k == pygame.K_SPACE: state["soft"] = True
                    elif k == pygame.K_UP: gain(5)
                    elif k == pygame.K_DOWN: gain(-5)
                elif e.type == pygame.JOYBUTTONDOWN:
                    b = e.button
                    state["pressed"] = f"tombol {b}"
                    if b in (Config.BTN_ESTOP, Config.BTN_DISARM): estop()
                    elif b == Config.BTN_ARM: do_arm()
                    elif b == Config.BTN_SOFT_STOP: state["soft"] = True
                    elif b == Config.BTN_PWR_UP: gain(5)
                    elif b == Config.BTN_PWR_DOWN: gain(-5)

            now = time.time()
            dt, t_prev = now - t_prev, now

            raw = 0.0
            if joy:
                n = joy.get_numaxes()
                g = lambda a: joy.get_axis(a) if a < n else 0.0
                raw = g(Config.AXIS_FWD)
                ch_x.stick = norm_axis(raw, centers[Config.AXIS_FWD], Config.INV_FWD)
                ch_r.stick = norm_axis(g(Config.AXIS_YAW), centers[Config.AXIS_YAW], Config.INV_YAW)
                ch_z.stick = norm_axis(g(Config.AXIS_UP), centers[Config.AXIS_UP], Config.INV_UP)
            else:
                kk = pygame.key.get_pressed()
                ch_x.stick = float(kk[pygame.K_w] - kk[pygame.K_s])
                ch_r.stick = float(kk[pygame.K_d] - kk[pygame.K_a])
                ch_z.stick = float(kk[pygame.K_r] - kk[pygame.K_f])

            if px.armed and not px.link_ok:
                print("[FAILSAFE] link putus")
                estop()

            if px.armed:
                x = ch_x.update(dt, state["gain"], state["soft"])
                r = ch_r.update(dt, state["gain"], state["soft"])
                z = ch_z.update(dt, state["gain"], state["soft"])
                if state["soft"] and max(abs(x), abs(r), abs(z)) < 1:
                    state["soft"] = False
            else:
                zero_all()
                x = r = z = 0.0

            px.send_control(x, 500 + z, r)       # dikirim terus tiap frame
            px.poll(now)
            h_out.append(px.pwm or 1500)

            if not px.armed:
                txt, col = "DISARMED - motor dipaksa netral oleh Pixhawk", GREEN
            elif state["soft"]:
                txt, col = "BERHENTI PELAN ...", ORANGE
            else:
                txt, col = "ARMED - dorong stick pelan", RED

            screen.fill(BG)
            card(screen, (20, 20, 880, 70), "STATUS", small)
            screen.blit(big.render(txt, True, col), (40, 45))

            card(screen, (20, 105, 330, 250), "PWM KELUAR (servo1)", small)
            gauge(screen, 185, 285, 120, px.pwm or 1500, font)
            card(screen, (365, 105, 535, 250), "GRAFIK PWM keluar", small)
            graph(screen, (380, 135, 505, 205), [(h_out, GREEN)], small)

            card(screen, (20, 370, 430, 220), "DATA", small)
            rows = [(f"Maju    : {ch_x.stick:+.2f}  -> {x:+5.0f}", WHITE),
                    (f"Yaw     : {ch_r.stick:+.2f}  -> {r:+5.0f}", WHITE),
                    (f"Naik    : {ch_z.stick:+.2f}  -> {z:+5.0f}", WHITE),
                    (f"Gain    : {state['gain']:.0f}% (max {Config.MAX_GAIN:.0f}%)", ORANGE),
                    (f"PWM out : {px.pwm} us", GREEN),
                    (f"Mode    : {px.mode}", CYAN)]
            for i, (t, c) in enumerate(rows):
                screen.blit(font.render(t, True, c), (35, 400 + i * 30))

            card(screen, (465, 370, 435, 220), "INDIKATOR & KONTROL", small)
            dot(screen, 485, 408, px.link_ok, "Link", small)
            dot(screen, 600, 408, px.armed, "ARMED", small)
            dot(screen, 740, 408, joy is not None, "Joystick", small)
            dot(screen, 485, 432, px.params_ok, "Parameter (SERVO=33, FS_GCS=2)", small)
            helps = ["Enter=ARM  Backspace/X=E-STOP  Spasi=stop pelan",
                     "Atas/Bawah=gain  W/S,A/D,R/F=gerak (tanpa joystick)",
                     f"Tombol terakhir: {state['pressed']}",
                     f"Pixhawk: {px.status[:48]}"]
            for i, t in enumerate(helps):
                screen.blit(small.render(t, True, GREY), (480, 462 + i * 24))

            joy_panel(screen, (20, 605, 880, 235), joy, small, font)
            pygame.display.flip()
            clock.tick(Config.FPS)
    finally:
        # apa pun yang terjadi: nol + disarm
        try:
            px.zero(10)
            px.arm(False)
        finally:
            pygame.quit()


if __name__ == "__main__":
    main()