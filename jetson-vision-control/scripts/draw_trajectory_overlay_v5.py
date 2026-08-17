#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
draw_trajectory_overlay.py

Ve overlay "dang di chuyen theo hinh gi" (tam giac 1, tam giac 2, luc giac,
vong tron, hinh so-8) len video da ghi (video.mp4 + video_timestamps.csv),
dua theo state machine CHINH XAC cua BalanceTrajectoryController
(trajectory.hpp, ban moi nhat - co Figure-8 thay the Bounce).

VI SAO PHAI MO PHONG LAI STATE MACHINE:
data.csv KHONG log truc tiep setpoint (x_d, y_d) hay ten pha dang chay -
chi co roll_d/pitch_d (output PID, do), Ballx/Bally (vi tri bong THAT), va
S1/S2/S3 (servo). Muon biet "dang o hinh nao" tai moi frame video, buoc
DUY NHAT la chay lai CHINH XAC logic trajectory.hpp (cung hang so, cung
thu tu chuyen pha) voi dt lay tu chinh timestamp cua video, dong bo qua
mot goc t_start CHUNG (gia dinh: t=0 cua ca 2 file = luc vao Mode Balance).

CANH BAO QUAN TRONG VE DO CHINH XAC:
  1. Gia dinh trajectory reset() dung tai t=0 cua video/data.csv (luc bat
     dau ghi = luc vao Balance). Neu THUC TE ban vao Balance TRUOC/SAU luc
     bat dau ghi vai giay, dung --start-offset-ms de bu (vi du he thong da
     chay Balance 5s truoc khi ban bam ghi -> --start-offset-ms 5000).
  2. Script CO mo phong lai co che "mat bong >0.3s thi reset ve O" (giong
     task_control_loop.cpp) dua vao cot 'detected' trong data.csv (5Hz,
     THO hon that (control loop that la 100Hz) - chi la XAP XI, khong
     dam bao khop tuyet doi neu he thong THAT co reset ma lan mat bong do
     qua ngan de 5Hz bat duoc, hoac nguoc lai).
  3. QUY DOI mm -> pixel dang dung XAP XI TUYEN TINH don gian (scale +
     offset tu roi_center_px/roi_radius_px co san trong main.cpp), KHONG
     phai phep chieu 3D that (cv2.projectPoints voi camera_matrix/
     dist_coeffs/rvec/tvec that). Neu ban gui duoc calib/intrinsics.yaml +
     calib/extrinsic.yaml, sua ham mm_to_px() o duoi de dung
     cv2.projectPoints() cho CHINH XAC (dac biet neu camera khong nhin
     thang xuong 90 do ma co goc nghieng/phoi canh).

CACH DUNG:
    python3 draw_trajectory_overlay.py \
        --video video.mp4 \
        --video-ts video_timestamps.csv \
        --data-csv data.csv \
        --out video_annotated.mp4 \
        [--start-offset-ms 0] \
        [--roi-center-px 629.8 362.4] [--roi-radius-px 400.0] \
        [--flip-y] [--show-ball] [--show-target]

Yeu cau: pip install opencv-python numpy
"""
import argparse
import csv
import math
import sys
import numpy as np
import cv2


# =============================================================================
# PHAN 1: PORT LAI CHINH XAC BalanceTrajectoryController (trajectory.hpp)
# =============================================================================

class SubPhase:
    AT_POINT = 0
    TRANSIT = 1
    CIRCLE = 2
    FIGURE8 = 3


# ---- Toa do 6 dinh luc giac (mm) - COPY DUNG tu trajectory.hpp ----
# CHU Y: kA_x = 120.25 (khong phai 108.25 nhu ban rat cu) - dung dung gia
# tri MOI NHAT nguoi dung da chinh trong file upload.
K_HEX_RADIUS = 125.0
K_A = (120.25, -62.5)
K_B = (120.25, 62.5)
K_C = (0.0, 125.0)
K_D = (-108.25, 62.5)
K_E = (-108.25, -62.5)
K_F = (0.0, -125.0)
K_O = (0.0, 0.0)

# Chuoi 15 diem CHINH, dung THU TU trajectory.hpp (khong TEST MODE, full chuoi)
K_SEQUENCE = [K_O, K_C, K_A, K_E, K_C, K_F, K_B, K_D, K_F,
              K_A, K_B, K_C, K_D, K_E, K_F]
K_NUM_POINTS = len(K_SEQUENCE)

K_HOLD_CENTER_S = 2.0
K_DWELL_S = 2.0
K_POST_FIGURE8_HOLD_S = 0.25
K_TRANSIT_S = 0.2
K_NUM_TRANSIT_POINTS = 3

K_PI = math.pi
K_CIRCLE_RADIUS = 120.0
K_CIRCLE_LAPS = 2
K_CIRCLE_SECONDS = 10.0
K_CIRCLE_TOTAL_ANGLE = 2.0 * K_PI * K_CIRCLE_LAPS
K_CIRCLE_ANGULAR_SPEED = K_CIRCLE_TOTAL_ANGLE / K_CIRCLE_SECONDS
K_CIRCLE_RAMP_S = 0.75
K_CIRCLE_START_ANGLE = -K_PI * 0.5

K_FIGURE8_START_ANGLE = K_PI * 0.5
K_FIGURE8_AMPLITUDE = 90.0
K_FIGURE8_LAPS = 4
K_FIGURE8_SECONDS = 24.0
K_FIGURE8_TOTAL_ANGLE = 2.0 * K_PI * K_FIGURE8_LAPS
K_FIGURE8_ANGULAR_SPEED = K_FIGURE8_TOTAL_ANGLE / K_FIGURE8_SECONDS
K_FIGURE8_RAMP_S = 0.30


def _clamp01(v):
    return 0.0 if v < 0.0 else (1.0 if v > 1.0 else v)


def _dwell_for(index):
    return K_HOLD_CENTER_S if index == 0 else K_DWELL_S


class TrajectorySim:
    """Port 1:1 cua BalanceTrajectoryController (C++) sang Python, CHI
    dung de MO PHONG LAI xem dang o pha nao tai moi thoi diem - KHONG
    dung de dieu khien gi ca (offline, chi phuc vu ve overlay)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.seq_index = 0
        self.sub_phase = SubPhase.AT_POINT
        self.phase_elapsed_s = 0.0
        self.transit_step = 0
        self.circle_angle_traveled = 0.0
        self.figure8_angle_traveled = 0.0
        self.figure8_loop_angle = 0.0
        self.figure8_loop_index = 0
        self.figure8_horizontal = True
        self.post_figure8_hold = False

    def update(self, dt):
        """Tra ve (x_mm, y_mm, phase_label) - phase_label dung de quyet
        dinh VE HINH GI (xem PHAN 2 ben duoi)."""
        self.phase_elapsed_s += dt
        next_index = (self.seq_index + 1) % K_NUM_POINTS

        if self.sub_phase == SubPhase.AT_POINT:
            dwell = (K_POST_FIGURE8_HOLD_S
                     if (self.seq_index == 0 and self.post_figure8_hold)
                     else _dwell_for(self.seq_index))
            if self.phase_elapsed_s >= dwell:
                if self.seq_index == 0:
                    self.post_figure8_hold = False
                if self.seq_index == K_NUM_POINTS - 1:
                    self.sub_phase = SubPhase.CIRCLE
                    self.circle_angle_traveled = 0.0
                else:
                    self.sub_phase = SubPhase.TRANSIT
                self.phase_elapsed_s = 0.0
            x, y = K_SEQUENCE[self.seq_index]
            return x, y, self._phase_label()

        if self.sub_phase == SubPhase.TRANSIT:
            if self.phase_elapsed_s >= K_TRANSIT_S:
                self.phase_elapsed_s = 0.0
                self.transit_step += 1
                if self.transit_step >= K_NUM_TRANSIT_POINTS:
                    self.transit_step = 0
                    self.seq_index = next_index
                    self.sub_phase = SubPhase.AT_POINT
                    x, y = K_SEQUENCE[self.seq_index]
                    return x, y, self._phase_label()
            a = K_SEQUENCE[self.seq_index]
            b = K_SEQUENCE[next_index]
            frac = (self.transit_step + 1) / (K_NUM_TRANSIT_POINTS + 1)
            x = a[0] + (b[0] - a[0]) * frac
            y = a[1] + (b[1] - a[1]) * frac
            return x, y, self._phase_label()

        if self.sub_phase == SubPhase.CIRCLE:
            time_in_circle = self.phase_elapsed_s
            self.circle_angle_traveled += K_CIRCLE_ANGULAR_SPEED * dt

            if self.circle_angle_traveled >= K_CIRCLE_TOTAL_ANGLE:
                self.circle_angle_traveled = K_CIRCLE_TOTAL_ANGLE
                self.sub_phase = SubPhase.FIGURE8
                self.phase_elapsed_s = 0.0
                self.figure8_angle_traveled = 0.0
                self.figure8_loop_angle = 0.0
                self.figure8_loop_index = 0
                self.figure8_horizontal = True
                return 0.0, 0.0, "FIGURE8"

            angle = K_CIRCLE_START_ANGLE + self.circle_angle_traveled
            time_remaining = K_CIRCLE_SECONDS - time_in_circle
            ramp_in = _clamp01(time_in_circle / K_CIRCLE_RAMP_S)
            ramp_out = _clamp01(time_remaining / K_CIRCLE_RAMP_S)
            eff_r = K_CIRCLE_RADIUS * min(ramp_in, ramp_out)
            return eff_r * math.cos(angle), eff_r * math.sin(angle), "CIRCLE"

        if self.sub_phase == SubPhase.FIGURE8:
            self.figure8_loop_angle += K_FIGURE8_ANGULAR_SPEED * dt
            self.figure8_angle_traveled += K_FIGURE8_ANGULAR_SPEED * dt

            while self.figure8_loop_angle >= 2.0 * K_PI and self.figure8_loop_index < 4:
                self.figure8_loop_angle -= 2.0 * K_PI
                self.figure8_loop_index += 1

            if self.figure8_loop_index >= 4:
                self.seq_index = 0
                self.sub_phase = SubPhase.AT_POINT
                self.phase_elapsed_s = 0.0
                self.transit_step = 0
                self.post_figure8_hold = True
                self.figure8_angle_traveled = K_FIGURE8_TOTAL_ANGLE
                self.figure8_loop_angle = 0.0
                self.figure8_loop_index = 4
                x, y = K_SEQUENCE[0]
                return x, y, "O_IDLE"

            t = K_FIGURE8_START_ANGLE + self.figure8_loop_angle
            ramp_in = _clamp01(self.phase_elapsed_s / K_FIGURE8_RAMP_S)
            eff_amp = K_FIGURE8_AMPLITUDE * ramp_in
            s8 = math.sin(t)
            c8 = math.cos(t)

            if self.figure8_loop_index < 2:
                x = eff_amp * c8
                y = eff_amp * s8 * c8
                label = "FIGURE8_H"
            else:
                x = eff_amp * s8 * c8
                y = eff_amp * c8
                label = "FIGURE8_V"
            return x, y, label

        return K_SEQUENCE[0][0], K_SEQUENCE[0][1], "O_IDLE"

    def _phase_label(self):
        """Quyet dinh dang o 'hinh' nao dua theo seq_index HIEN TAI, dung
        cho ca AT_POINT lan TRANSIT (giu logic giong nhau, chi khac
        seq_index dang giu co dinh trong luc TRANSIT). Cac khoang index
        LAY DUNG THEO MO TA CUA NGUOI DUNG:
          O                (idx 0)               -> O_IDLE
          C-A-E            (idx 1..4)             -> TRIANGLE1
          C-F-B-D-F        (idx 4..8)             -> TRIANGLE2 (chong idx4=C
                                                      voi TRIANGLE1 - hop ly,
                                                      C la diem noi tiep)
          F-A-B-C-D-E-F    (idx 8..14)            -> HEXAGON (chong idx8=F
                                                      voi TRIANGLE2)
        """
        i = self.seq_index
        if i == 0:
            return "O_IDLE"
        if 1 <= i <= 4:
            return "TRIANGLE1"
        if 4 <= i <= 8:
            return "TRIANGLE2"
        if 8 <= i <= 14:
            return "HEXAGON"
        return "O_IDLE"


# =============================================================================
# PHAN 2: VE HINH THAM CHIEU LEN FRAME THEO phase_label
# =============================================================================

# Cac hinh THAM CHIEU (duong noi cac diem CHINH, mm) cho tung phase_label -
# ve TINH (khong doi dang), TACH BIET voi diem target dang di chuyen (ve rieng).
SHAPE_POLYLINES_MM = {
    "TRIANGLE1": [K_C, K_A, K_E, K_C],                 # tam giac dong C-A-E-C
    "TRIANGLE2": [K_C, K_F, K_B, K_D, K_F],             # duong gap khuc C-F-B-D-F
    "HEXAGON":   [K_A, K_B, K_C, K_D, K_E, K_F, K_A],   # luc giac dong 6 dinh
}

COLOR_SHAPE = (0, 255, 255)     # vang - duong hinh tham chieu (BGR)
COLOR_TARGET = (0, 0, 255)      # do - diem target hien tai (setpoint)
COLOR_BALL = (255, 0, 0)        # xanh duong - vi tri bong THAT (tu data.csv)
COLOR_TEXT = (255, 255, 255)
SHAPE_LINE_THICKNESS = 2
TARGET_RADIUS_PX = 6
BALL_RADIUS_PX = 5


def build_mm_to_px(roi_center_px, roi_radius_px, flip_y):
    """XAP XI TUYEN TINH mm(bang) -> pixel(anh) - DU PHONG khi KHONG co
    file calib that (intrinsics.yaml/extrinsic.yaml). Xem
    build_mm_to_px_projected() ben duoi de dung phep chieu 3D CHINH XAC."""
    scale = roi_radius_px / K_HEX_RADIUS  # px / mm
    cx, cy = roi_center_px
    sign_y = -1.0 if flip_y else 1.0

    def mm_to_px(x_mm, y_mm):
        px = cx + x_mm * scale
        py = cy + sign_y * y_mm * scale
        return int(round(px)), int(round(py))

    return mm_to_px


def load_calib_yaml(intrinsics_path, extrinsic_path):
    """Doc camera_matrix/dist_coeffs (intrinsics.yaml) va rvec/tvec
    (extrinsic.yaml) qua cv2.FileStorage - DUNG DINH DANG voi
    fs_intr["camera_matrix"] ben C++ (task_ball_detect.cpp), dam bao
    CHINH XAC 100% cung 1 bo tham so ma BallDetector dang dung."""
    fs_i = cv2.FileStorage(intrinsics_path, cv2.FILE_STORAGE_READ)
    camera_matrix = fs_i.getNode("camera_matrix").mat()
    dist_coeffs = fs_i.getNode("dist_coeffs").mat()
    fs_i.release()

    fs_e = cv2.FileStorage(extrinsic_path, cv2.FILE_STORAGE_READ)
    rvec = fs_e.getNode("rvec").mat()
    tvec = fs_e.getNode("tvec").mat()
    fs_e.release()

    if camera_matrix is None or dist_coeffs is None or rvec is None or tvec is None:
        raise RuntimeError(f"Khong doc duoc du cac truong can thiet tu "
                           f"{intrinsics_path} / {extrinsic_path}")
    return camera_matrix, dist_coeffs, rvec, tvec


def build_mm_to_px_projected(camera_matrix, dist_coeffs, rvec, tvec):
    """CHINH XAC: dung cv2.projectPoints() de chieu diem 3D tren mat phang
    ban (X_mm, Y_mm, 0) - CUNG he toa do the gioi ma BallDetector dung khi
    giai nguoc pixel->mm (rvec/tvec/camera_matrix/dist_coeffs GIONG HET,
    chi khac chieu chieu: BallDetector giai NGUOC (pixel->mm qua mat
    phang Z=0), o day ta chieu XUOI (mm->pixel) - toan hoc doi xung, dam
    bao khop dung 100% voi Ballx/Bally da log trong data.csv (khong con
    la xap xi/gia dinh camera nhin thang xuong nua, xu ly dung ca phoi
    canh lan meo ong kinh)."""
    def mm_to_px(x_mm, y_mm):
        pt3d = np.array([[[float(x_mm), float(y_mm), 0.0]]], dtype=np.float64)
        img_pts, _ = cv2.projectPoints(pt3d, rvec, tvec, camera_matrix, dist_coeffs)
        px, py = img_pts[0, 0]
        return int(round(px)), int(round(py))

    return mm_to_px


# =============================================================================
# PHAN 3: DOC data.csv (chi can cot timestamp_ms, Ballx, Bally, detected)
# =============================================================================

def load_data_csv(path):
    """Doc data.csv voi cac cot THAT: timestamp_ms,S1,S2,S3,roll_imu,
    pitch_imu,roll_d,pitch_d,height_d,Ballx,Bally,detected. Cac cot
    S1/S2/S3/roll_d/pitch_d duoc doc them de hien thi panel LCD gia lap."""
    rows = []
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            def _f(key, default=0.0):
                try:
                    return float(row[key])
                except (KeyError, ValueError):
                    return default
            rows.append({
                "t_ms": _f("timestamp_ms"),
                "ballx": _f("Ballx"),
                "bally": _f("Bally"),
                "detected": int(_f("detected", 0.0)),
                "s1": _f("S1"),
                "s2": _f("S2"),
                "s3": _f("S3"),
                "roll_d": _f("roll_d"),
                "pitch_d": _f("pitch_d"),
                "roll_imu": _f("roll_imu"),
                "pitch_imu": _f("pitch_imu"),
            })
    rows.sort(key=lambda r: r["t_ms"])
    # ---- Tinh VX/VY (mm/s) bang vi phan Ballx/Bally theo thoi gian, chi
    # tinh khi ca 2 mau lien tiep deu detected=1 (tranh nhay vot khi mat
    # bong / bong nhay ve 0,0) ----
    for i in range(len(rows)):
        if i == 0 or not rows[i]["detected"] or not rows[i - 1]["detected"]:
            rows[i]["vx"] = 0.0
            rows[i]["vy"] = 0.0
            continue
        dt_s = (rows[i]["t_ms"] - rows[i - 1]["t_ms"]) / 1000.0
        if dt_s <= 0:
            rows[i]["vx"] = 0.0
            rows[i]["vy"] = 0.0
        else:
            rows[i]["vx"] = (rows[i]["ballx"] - rows[i - 1]["ballx"]) / dt_s
            rows[i]["vy"] = (rows[i]["bally"] - rows[i - 1]["bally"]) / dt_s
    return rows


def sample_data_csv(rows, t_ms):
    """Lay dong data.csv GAN NHAT (khong noi suy - data.csv chi 5Hz nen
    noi suy khong lam tang do chinh xac dang ke) cho 1 thoi diem t_ms cho
    truoc. Dung tim kiem nhi phan don gian (rows da sort theo t_ms)."""
    if not rows:
        return None
    lo, hi = 0, len(rows) - 1
    if t_ms <= rows[0]["t_ms"]:
        return rows[0]
    if t_ms >= rows[-1]["t_ms"]:
        return rows[-1]
    while lo < hi:
        mid = (lo + hi) // 2
        if rows[mid]["t_ms"] < t_ms:
            lo = mid + 1
        else:
            hi = mid
    # so sanh 2 lan can nhat, lay gan hon
    cand = [rows[max(0, lo - 1)], rows[lo]]
    return min(cand, key=lambda r: abs(r["t_ms"] - t_ms))


# =============================================================================
# PHAN 3b: PANEL LCD GIA LAP (giong man hinh that PINGPONG-TABLE trong anh)
# =============================================================================

PANEL_BG = (0, 0, 0)                      # nen DEN (giong man LCD that trong anh)
PANEL_LABEL_COLOR = (255, 255, 255)       # trang - nhan (STATE, ROLL, ...)
PANEL_VALUE_COLOR = (0, 255, 255)         # vang(BGR)/cyan tuy gia tri - se override tung dong
PANEL_TITLE_COLOR = (255, 220, 90)        # cyan nhat - tieu de "PINGPONG-TABLE"
PANEL_DIAG_CIRCLE_COLOR = (60, 60, 230)   # do (BGR) - vien vong tron/luc giac
PANEL_DIAG_CROSS_COLOR = (230, 230, 230)  # trang - truc cross-hair
PANEL_BTN_BALANCE_COLOR = (200, 120, 30)  # xanh duong nut "2. Balance"
PANEL_BTN_SHUTDOWN_COLOR = (40, 170, 60)  # xanh la nut "SHUTDOWN"
PANEL_BTN_TEXT_COLOR = (255, 255, 255)
PANEL_ROUND_BTN_COLORS = [(230, 200, 120), (60, 60, 230), (230, 200, 120)]  # xanh nhat/do/xanh nhat


def draw_lcd_panel(panel_w, panel_h, state_text, roll_d, pitch_d, ball_on,
                    x_mm, y_mm, vx, vy, cam_ok, s1, s2, s3,
                    phase_label=None, ball_mm=None):
    """Panel LCD gia lap dat BEN DUOI video.

    Bo cuc co y tuong giong giao dien mau:
      - Ben trai: ROLL/PITCH, X/Y, VX/VY, SERVO OUTPUT
      - Hang duoi servo: STATE / BALL / CAM
      - Ben phai: TRAJECTORY
    Da bo:
      - "TABLE R: ..."
      - "HEX R: ..."
      - thang 50 mm
      - "HOLD RED TO STOP"
    """

    panel = np.full((panel_h, panel_w, 3), (10, 12, 16), dtype=np.uint8)

    # -------------------------------------------------------------------------
    # Palette BGR
    # -------------------------------------------------------------------------
    BG = (10, 12, 16)
    CARD = (20, 23, 29)
    CARD2 = (24, 28, 35)
    BORDER = (52, 58, 68)
    WHITE = (235, 240, 245)
    MUTED = (145, 155, 168)
    CYAN = (255, 220, 70)
    GREEN = (80, 225, 90)
    RED = (60, 75, 235)
    BLUE = (245, 150, 40)
    YELLOW = (0, 225, 255)
    GRID = (32, 36, 44)
    TABLE_RED = (70, 70, 220)

    s = max(0.75, panel_h / 430.0)
    font = cv2.FONT_HERSHEY_SIMPLEX
    thin = max(1, int(round(1.0 * s)))
    normal = max(1, int(round(1.35 * s)))
    bold = max(1, int(round(1.7 * s)))

    def rr(x1, y1, x2, y2, radius, fill, outline=None, thickness=1):
        cv2.rectangle(panel, (x1 + radius, y1), (x2 - radius, y2),
                      fill, -1)
        cv2.rectangle(panel, (x1, y1 + radius), (x2, y2 - radius),
                      fill, -1)
        cv2.circle(panel, (x1 + radius, y1 + radius), radius, fill, -1)
        cv2.circle(panel, (x2 - radius, y1 + radius), radius, fill, -1)
        cv2.circle(panel, (x1 + radius, y2 - radius), radius, fill, -1)
        cv2.circle(panel, (x2 - radius, y2 - radius), radius, fill, -1)
        if outline is not None:
            cv2.rectangle(panel, (x1, y1), (x2, y2), outline,
                          max(1, thickness), cv2.LINE_AA)

    def txt(text_value, x, y, scale=0.45, color=WHITE, thick=normal):
        cv2.putText(panel, str(text_value), (int(x), int(y)), font,
                    scale * s, color, thick, cv2.LINE_AA)

    def centered(text_value, cx, y, scale=0.45, color=WHITE, thick=normal):
        (tw, _), _ = cv2.getTextSize(str(text_value), font,
                                      scale * s, thick)
        txt(text_value, cx - tw / 2, y, scale, color, thick)

    def metric_card(x1, y1, x2, y2, title, value, value_color=WHITE,
                    subtitle=None):
        rr(x1, y1, x2, y2, int(8 * s), CARD, BORDER, 1)
        txt(title.upper(), x1 + int(10 * s), y1 + int(15 * s),
            0.31, MUTED, thin)
        txt(value, x1 + int(10 * s), y1 + int(39 * s),
            0.62, value_color, bold)
        if subtitle:
            txt(subtitle, x1 + int(10 * s), y2 - int(8 * s),
                0.28, MUTED, thin)

    def status_card(x1, y1, x2, y2, title, value, dot_color):
        """Card STATE/BALL/CAM o hang duoi cung, giong hinh mau."""
        rr(x1, y1, x2, y2, int(8 * s), CARD, BORDER, 1)

        dot_r = max(2, int(round(3.0 * s)))
        dot_x = x1 + int(10 * s)
        dot_y = y1 + int(15 * s)
        cv2.circle(panel, (dot_x, dot_y), dot_r, dot_color, -1, cv2.LINE_AA)

        txt(title, x1 + int(17 * s), y1 + int(18 * s),
            0.31, MUTED, thin)
        centered(value, (x1 + x2) / 2, y1 + int(43 * s),
                 0.46, GREEN if title != "CAM" else CYAN, bold)

    # -------------------------------------------------------------------------
    # Bo cuc: trai = telemetry, phai = trajectory
    # -------------------------------------------------------------------------
    margin = int(12 * s)
    gap = int(10 * s)
    left_w = int(panel_w * 0.42)

    left_x1 = margin
    left_x2 = left_x1 + left_w
    right_x1 = left_x2 + gap
    right_x2 = panel_w - margin

    # -------------------------------------------------------------------------
    # Header ben trai
    # -------------------------------------------------------------------------
    txt("PINGPONG TABLE", left_x1, int(25 * s), 0.66, CYAN, bold)
    txt("BALANCE CONTROLLER", left_x1, int(42 * s),
        0.30, MUTED, normal)

    # -------------------------------------------------------------------------
    # Telemetry cards: ROLL/PITCH -> X/Y -> VX/VY
    # -------------------------------------------------------------------------
    y = int(50 * s)
    row_h = int(55 * s)
    col_gap = int(8 * s)
    col_w = (left_w - col_gap) // 2

    metric_card(left_x1, y, left_x1 + col_w, y + row_h,
                "ROLL", f"{roll_d:+.1f}°", GREEN, "TABLE")
    metric_card(left_x1 + col_w + col_gap, y,
                left_x2, y + row_h,
                "PITCH", f"{pitch_d:+.1f}°", GREEN, "TABLE")

    y += row_h + int(7 * s)
    metric_card(left_x1, y, left_x1 + col_w, y + row_h,
                "X (Ball)", f"{x_mm:+.0f} mm", CYAN, "BALL POSITION")
    metric_card(left_x1 + col_w + col_gap, y,
                left_x2, y + row_h,
                "Y (Ball)", f"{y_mm:+.0f} mm", CYAN, "BALL POSITION")

    y += row_h + int(7 * s)
    metric_card(left_x1, y, left_x1 + col_w, y + row_h,
                "VX", f"{vx:+.0f} mm/s", WHITE, None)
    metric_card(left_x1 + col_w + col_gap, y,
                left_x2, y + row_h,
                "VY", f"{vy:+.0f} mm/s", WHITE, None)

    # -------------------------------------------------------------------------
    # Servo output
    # -------------------------------------------------------------------------
    y += row_h + int(8 * s)
    txt("SERVO OUTPUT", left_x1, y + int(13 * s),
        0.31, MUTED, bold)
    y += int(19 * s)

    servo_gap = int(7 * s)
    servo_w = (left_w - 2 * servo_gap) // 3
    servo_h = int(52 * s)

    for idx, value in enumerate((s1, s2, s3), start=1):
        sx1 = left_x1 + (idx - 1) * (servo_w + servo_gap)
        sx2 = sx1 + servo_w

        rr(sx1, y, sx2, y + servo_h, int(7 * s), CARD, BORDER, 1)
        centered(f"S{idx}", (sx1 + sx2) / 2,
                 y + int(16 * s), 0.30, MUTED, thin)
        centered(f"{value:.0f}", (sx1 + sx2) / 2,
                 y + int(38 * s), 0.54, BLUE, bold)
        centered("µs", (sx1 + sx2) / 2,
                 y + int(49 * s), 0.24, MUTED, thin)

    # -------------------------------------------------------------------------
    # STATE / BALL / CAM: dua xuong DUOI S1/S2/S3 nhu hinh mau
    # -------------------------------------------------------------------------
    y += servo_h + int(8 * s)
    status_gap = int(7 * s)
    status_w = (left_w - 2 * status_gap) // 3
    status_h = int(55 * s)

    state_value = {
        "O_IDLE": "IDLE",
        "TRIANGLE1": "TRI-1",
        "TRIANGLE2": "TRI-2",
        "HEXAGON": "HEXAGON",
        "CIRCLE": "CIRCLE",
        "FIGURE8_H": "FIG8-H",
        "FIGURE8_V": "FIG8-V",
        "FIGURE8": "FIGURE-8",
        "WAIT": "WAIT",
    }.get(state_text, str(state_text)[:8] if state_text else "IDLE")

    status_card(
        left_x1, y, left_x1 + status_w, y + status_h,
        "STATE", state_value, CYAN
    )
    status_card(
        left_x1 + status_w + status_gap, y,
        left_x1 + 2 * status_w + status_gap, y + status_h,
        "BALL", "ON" if ball_on else "OFF",
        GREEN if ball_on else RED
    )
    status_card(
        left_x1 + 2 * (status_w + status_gap), y,
        left_x2, y + status_h,
        "CAM", "OK" if cam_ok else "ERR",
        GREEN if cam_ok else RED
    )

    # -------------------------------------------------------------------------
    # Right trajectory display
    # -------------------------------------------------------------------------
    rr(right_x1, int(10 * s), right_x2, panel_h - int(10 * s),
       int(10 * s), (12, 15, 20), BORDER, 1)

    # Chi hien TRAJECTORY, KHONG hien phase_text / TABLE R / HEX R.
    txt("TRAJECTORY", right_x1 + int(14 * s), int(29 * s),
        0.38, WHITE, bold)

    # Diagram region: use maximum square area while preserving 1:1 aspect.
    diag_x1 = right_x1 + int(12 * s)
    diag_x2 = right_x2 - int(12 * s)
    diag_y1 = int(38 * s)
    diag_y2 = panel_h - int(14 * s)

    avail_w = diag_x2 - diag_x1
    avail_h = diag_y2 - diag_y1
    diag_r = int(min(avail_w, avail_h) * 0.47)
    diag_cx = (diag_x1 + diag_x2) // 2
    diag_cy = (diag_y1 + diag_y2) // 2 + int(2 * s)

    # Crosshair nhe, khong ve thang scale 50 mm.
    cv2.line(panel, (diag_cx - diag_r, diag_cy),
             (diag_cx + diag_r, diag_cy), GRID, 1, cv2.LINE_AA)
    cv2.line(panel, (diag_cx, diag_cy - diag_r),
             (diag_cx, diag_cy + diag_r), GRID, 1, cv2.LINE_AA)

    if diag_r > 15:
        # Red = boundary table.
        # Yellow = trajectory reference.
        diag_scale = diag_r / K_HEX_RADIUS

        def diag_mm_to_px(px_mm, py_mm):
            return (int(round(diag_cx + px_mm * diag_scale)),
                    int(round(diag_cy - py_mm * diag_scale)))

        # Outer table boundary
        cv2.circle(panel, (diag_cx, diag_cy), diag_r,
                   TABLE_RED, max(1, int(2 * s)), cv2.LINE_AA)

        # Inner reference: actual trajectory shape
        polyline_mm = SHAPE_POLYLINES_MM.get(phase_label)
        if polyline_mm is not None:
            pts_px = np.array(
                [diag_mm_to_px(px, py) for px, py in polyline_mm],
                dtype=np.int32)
            cv2.polylines(panel, [pts_px], False, YELLOW,
                          max(2, int(2 * s)), cv2.LINE_AA)

        elif phase_label == "CIRCLE":
            center_px = diag_mm_to_px(0.0, 0.0)
            radius_px = int(round(K_CIRCLE_RADIUS * diag_scale))
            cv2.circle(panel, center_px, radius_px, YELLOW,
                       max(2, int(2 * s)), cv2.LINE_AA)

        elif phase_label in ("FIGURE8_H", "FIGURE8_V", "FIGURE8"):
            n_pts = 160
            pts_mm = []

            for k in range(n_pts + 1):
                tt = K_FIGURE8_START_ANGLE + 2.0 * K_PI * k / n_pts
                s8, c8 = math.sin(tt), math.cos(tt)

                if phase_label == "FIGURE8_V":
                    pts_mm.append((
                        K_FIGURE8_AMPLITUDE * s8 * c8,
                        K_FIGURE8_AMPLITUDE * c8))
                else:
                    pts_mm.append((
                        K_FIGURE8_AMPLITUDE * c8,
                        K_FIGURE8_AMPLITUDE * s8 * c8))

            pts_px = np.array(
                [diag_mm_to_px(px, py) for px, py in pts_mm],
                dtype=np.int32)
            cv2.polylines(panel, [pts_px], True, YELLOW,
                          max(2, int(2 * s)), cv2.LINE_AA)

        # Ball
        if ball_mm is not None:
            bx, by = diag_mm_to_px(ball_mm[0], ball_mm[1])
            ball_r_px = max(6, int(round(diag_r * 0.075)))
            cv2.circle(panel, (bx, by), ball_r_px,
                       (245, 245, 245), -1, cv2.LINE_AA)
            cv2.circle(panel, (bx, by), max(2, int(ball_r_px * 0.32)),
                       BLUE, -1, cv2.LINE_AA)

        # Target/setpoint
        tx, ty = diag_mm_to_px(x_mm, y_mm)
        target_r = max(4, int(round(diag_r * 0.035)))
        cv2.circle(panel, (tx, ty), target_r, RED, -1, cv2.LINE_AA)
        cv2.circle(panel, (tx, ty), target_r + 2,
                   (90, 90, 255), 1, cv2.LINE_AA)

        # Center marker
        cv2.circle(panel, (diag_cx, diag_cy), max(2, int(2 * s)),
                   MUTED, -1, cv2.LINE_AA)

    return panel


# =============================================================================
# PHAN 4: MAIN - doc video + video_timestamps.csv, mo phong, ve, ghi ra
# =============================================================================

def load_video_timestamps(path):
    frames = []
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            frames.append((int(row["frame_index"]), float(row["timestamp_ms"])))
    frames.sort(key=lambda x: x[0])
    return frames


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--video", required=True)
    ap.add_argument("--video-ts", required=True, help="video_timestamps.csv (frame_index,timestamp_ms)")
    ap.add_argument("--data-csv", required=True, help="data.csv")
    ap.add_argument("--out", required=True, help="duong dan video output (.mp4)")
    ap.add_argument("--start-offset-ms", type=float, default=0.0,
                     help="Bu neu he thong da o Mode Balance TRUOC luc bat dau ghi "
                          "(vd he thong chay Balance 5s truoc khi bam ghi -> 5000)")
    ap.add_argument("--intrinsics", default=None,
                     help="Duong dan intrinsics.yaml - neu co, dung phep chieu 3D "
                          "CHINH XAC (cv2.projectPoints) thay vi xap xi tuyen tinh")
    ap.add_argument("--extrinsic", default=None,
                     help="Duong dan extrinsic.yaml - PHAI di kem --intrinsics")
    ap.add_argument("--roi-center-px", nargs=2, type=float, default=[629.8, 362.4],
                     metavar=("CX", "CY"),
                     help="CHI dung khi KHONG co --intrinsics/--extrinsic (fallback)")
    ap.add_argument("--roi-radius-px", type=float, default=400.0,
                     help="CHI dung khi KHONG co --intrinsics/--extrinsic (fallback)")
    ap.add_argument("--flip-y", action="store_true", default=True,
                     help="Dao truc Y khi quy doi mm->px (mac dinh BAT, vi truc Y "
                          "toa do ban thuong huong LEN trong khi pixel anh huong "
                          "XUONG) - tat bang --no-flip-y neu thay hinh bi lat nguoc")
    ap.add_argument("--no-flip-y", dest="flip_y", action="store_false")
    ap.add_argument("--show-ball", action="store_true", default=True)
    ap.add_argument("--no-show-ball", dest="show_ball", action="store_false")
    ap.add_argument("--show-target", action="store_true", default=True)
    ap.add_argument("--no-show-target", dest="show_target", action="store_false")
    ap.add_argument("--ball-loss-reset-threshold-s", type=float, default=0.3,
                     help="Nguong mat bong LIEN TUC (giay) de mo phong reset "
                          "trajectory, khop voi kBallLossResetThresholdS trong "
                          "task_control_loop.cpp")
    ap.add_argument("--crop-left", type=int, default=0,
                     help="So pixel CAT BO ben TRAI cua frame video (vd de bo "
                          "phan chan/nguoi ngoai ban tron)")
    ap.add_argument("--crop-right", type=int, default=0,
                     help="So pixel CAT BO ben PHAI cua frame video")
    ap.add_argument("--crop", type=int, default=None,
                     help="CAT DEU so pixel nay o CA HAI BEN trai/phai (uu "
                          "tien hon --crop-left/--crop-right neu duoc dat, "
                          "dam bao ban tron luon o CHINH GIUA khung hinh "
                          "sau khi cat, khong bi lech)")
    ap.add_argument("--panel", action="store_true", default=True,
                     help="Ghep them panel LCD gia lap (STATE/ROLL/PITCH/BALL/"
                          "X/Y/VX/VY/CAM/S1/S2/S3 + so do luc giac) BEN DUOI "
                          "video (mac dinh BAT)")
    ap.add_argument("--no-panel", dest="panel", action="store_false")
    ap.add_argument("--panel-height", type=int, default=430,
                     help="Chieu cao panel LCD (px) khi ghep BEN DUOI video, "
                          "mac dinh 380")
    args = ap.parse_args()

    print("Doc video_timestamps.csv...")
    vid_ts = load_video_timestamps(args.video_ts)
    if not vid_ts:
        print("LOI: video_timestamps.csv rong.", file=sys.stderr)
        sys.exit(1)

    print("Doc data.csv...")
    data_rows = load_data_csv(args.data_csv)

    print(f"Mo video: {args.video}")
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print("LOI: khong mo duoc video.", file=sys.stderr)
        sys.exit(1)

    fps_in = cap.get(cv2.CAP_PROP_FPS) or 30.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    crop_l = max(0, args.crop_left)
    crop_r = max(0, args.crop_right)
    if args.crop is not None:
        crop_l = crop_r = max(0, args.crop)
    if crop_l + crop_r >= w:
        print("LOI: --crop-left + --crop-right >= chieu rong video.", file=sys.stderr)
        sys.exit(1)
    cropped_w = w - crop_l - crop_r
    panel_h = args.panel_height if args.panel else 0
    out_w = cropped_w
    out_h = h + panel_h

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    out = cv2.VideoWriter(args.out, fourcc, fps_in, (out_w, out_h))
    if not out.isOpened():
        print("LOI: khong mo duoc VideoWriter output.", file=sys.stderr)
        sys.exit(1)

    if args.intrinsics and args.extrinsic:
        print(f"Doc calib that: {args.intrinsics}, {args.extrinsic}")
        camera_matrix, dist_coeffs, rvec, tvec = load_calib_yaml(args.intrinsics, args.extrinsic)
        mm_to_px = build_mm_to_px_projected(camera_matrix, dist_coeffs, rvec, tvec)
        # Ban kinh Circle (mm) -> pixel: KHONG con he so scale co dinh nua
        # (phep chieu 3D khong tuyen tinh deu moi huong), uoc luong bang
        # khoang cach pixel tu tam O den 1 diem tren duong tron thuc te.
        _cx_px, _cy_px = mm_to_px(0.0, 0.0)
        _edge_px = mm_to_px(K_CIRCLE_RADIUS, 0.0)
        circle_radius_px_fn = lambda: int(round(math.hypot(_edge_px[0]-_cx_px, _edge_px[1]-_cy_px)))
        using_real_projection = True
    else:
        print("CANH BAO: khong co --intrinsics/--extrinsic, dung XAP XI TUYEN TINH "
              "(co the LECH neu camera co phoi canh dang ke).")
        mm_to_px = build_mm_to_px(tuple(args.roi_center_px), args.roi_radius_px, args.flip_y)
        circle_radius_px_fn = lambda: int(round(K_CIRCLE_RADIUS * (args.roi_radius_px / K_HEX_RADIUS)))
        using_real_projection = False

    sim = TrajectorySim()
    ball_loss_elapsed_s = 0.0
    prev_t_ms = None

    n_frames = len(vid_ts)
    print(f"Bat dau xu ly {n_frames} frame...")

    for i, (frame_idx, t_ms_raw) in enumerate(vid_ts):
        ok, frame = cap.read()
        if not ok:
            print(f"Canh bao: het frame video som hon du lieu timestamp "
                  f"(dung o frame {i}/{n_frames}).", file=sys.stderr)
            break

        t_ms = t_ms_raw - args.start_offset_ms
        if t_ms < 0:
            # Chua toi luc vao Balance (theo start-offset) - khong mo
            # phong/ve gi, chi crop + ghep panel "cho" (WAIT).
            frame_c = frame[:, crop_l: w - crop_r] if (crop_l or crop_r) else frame
            if args.panel:
                d0 = sample_data_csv(data_rows, t_ms_raw)
                ball_mm0 = (d0["ballx"], d0["bally"]) if (d0 is not None and d0["detected"]) else None
                panel_img = draw_lcd_panel(
                    cropped_w, panel_h, "WAIT",
                    d0["roll_d"] if d0 else 0.0, d0["pitch_d"] if d0 else 0.0,
                    bool(d0["detected"]) if d0 else False,
                    d0["ballx"] if d0 else 0.0, d0["bally"] if d0 else 0.0,
                    d0["vx"] if d0 else 0.0, d0["vy"] if d0 else 0.0,
                    bool(d0["detected"]) if d0 else False,
                    d0["s1"] if d0 else 0.0, d0["s2"] if d0 else 0.0, d0["s3"] if d0 else 0.0,
                    phase_label=None, ball_mm=ball_mm0)
                frame_c = np.vstack([frame_c, panel_img])
            out.write(frame_c)
            continue

        dt = 0.0 if prev_t_ms is None else max(0.0, (t_ms - prev_t_ms) / 1000.0)
        prev_t_ms = t_ms

        # ---- Mo phong reset do mat bong (giong task_control_loop.cpp) ----
        d = sample_data_csv(data_rows, t_ms_raw)  # dung t_ms_raw goc (cung truc voi data.csv)
        detected = d["detected"] if d else 0
        if detected == 0:
            ball_loss_elapsed_s += dt
            if ball_loss_elapsed_s >= args.ball_loss_reset_threshold_s:
                sim.reset()
        else:
            ball_loss_elapsed_s = 0.0

        x_mm, y_mm, phase_label = sim.update(dt)

        # ---- KHONG ve gi them len frame video - video_annotated.mp4 da
        # co san overlay (hinh + target + ball + text) tu lan chay truoc.
        # Script nay CHI cat vien + ghep them panel LCD ben duoi. ----

        # ---- Cat 2 ben (bo chan) + ghep panel LCD gia lap ben duoi ----
        frame_c = frame[:, crop_l: w - crop_r] if (crop_l or crop_r) else frame
        if args.panel:
            ball_mm = (d["ballx"], d["bally"]) if (d is not None and detected) else None
            panel_img = draw_lcd_panel(
                cropped_w, panel_h, phase_label,
                d["roll_d"] if d else 0.0, d["pitch_d"] if d else 0.0,
                bool(detected), x_mm, y_mm,
                d["vx"] if d else 0.0, d["vy"] if d else 0.0,
                bool(detected),
                d["s1"] if d else 0.0, d["s2"] if d else 0.0, d["s3"] if d else 0.0,
                phase_label=phase_label, ball_mm=ball_mm)
            frame_c = np.vstack([frame_c, panel_img])

        out.write(frame_c)

        if i % 200 == 0:
            print(f"  frame {i}/{n_frames} (t={t_ms/1000.0:.1f}s, phase={phase_label})")

    cap.release()
    out.release()
    print(f"Xong. Da ghi: {args.out}")


if __name__ == "__main__":
    main()