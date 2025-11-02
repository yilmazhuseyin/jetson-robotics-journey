import cv2
import time
import re
import collections
import subprocess
import logging
import os
import math
import threading
from typing import Tuple, Optional

from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio

# ----------------------------
# Configuration
# ----------------------------

# OCR mode / plates
ALLOW_ALPHANUMERIC = True
ALLOWED_PLATES = {"JETSON"}
MIN_PLATE_LEN = 4

# OCR knobs
UPSCALE_HEIGHT = 120        
FUZZY_MAX_DIST = 1            
OCR_MAX_COMBOS_PER_FRAME = 2 

# Preferred OCR config (progressive fallback will try others as needed)
OEM_PREF = 1                 
PSM_PREF = 7                  

# Camera / ROI
PLATE_ROI = (0.25, 0.30, 0.50, 0.40)
CAM_INDEX = 0
USE_V4L2 = True
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
TARGET_FPS = 30

# Drop-late frames
DROP_LATE_FRAMES = True

# (Optional) lock exposure for stability (requires v4l2-ctl)
LOCK_CAMERA_EXPOSURE = False
EXPOSURE_ABS = 200
GAIN_VALUE = 0

# Match debouncing
MATCH_FRAMES = 1

# Gate cycle parameters
OPEN_SWEEP_DEG = 220
HOLD_OPEN_SECONDS = 5.0
OPEN_DURATION_S = 3.0
CLOSE_DURATION_S = 3.0

# Servo update thread cadence
APPLY_HZ = 100
APPLY_INTERVAL_S = 1.0 / APPLY_HZ

# Servo/PWM config
FREQ_HZ = 50
MIN_US, MAX_US = 1000, 2000

# Closed pose (adjust to your mechanics)
LEFT_CLOSED = 30.0
RIGHT_CLOSED = 150.0

# Tesseract binary path (CLI fallback)
TESSERACT_BIN = "/usr/bin/tesseract"

# ----------------------------
# Logging
# ----------------------------

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)

# ----------------------------
# Helpers
# ----------------------------

def clamp_angle(a: float) -> float:
    return max(0.0, min(180.0, float(a)))

def open_targets_from_closed() -> Tuple[float, float]:
    left_open = clamp_angle(LEFT_CLOSED + OPEN_SWEEP_DEG)
    right_open = clamp_angle(RIGHT_CLOSED - OPEN_SWEEP_DEG)
    return left_open, right_open

def ease_in_out_cosine(t: float) -> float:
    if t <= 0.0: return 0.0
    if t >= 1.0: return 1.0
    return 0.5 * (1.0 - math.cos(math.pi * t))

def safe_run_v4l2ctl(device: str = "/dev/video0"):
    if not LOCK_CAMERA_EXPOSURE:
        return
    try:
        subprocess.run(
            ["v4l2-ctl", "-d", device, "--set-ctrl=auto_exposure=1",
             f"--set-ctrl=exposure_time_absolute={EXPOSURE_ABS}",
             f"--set-ctrl=gain={GAIN_VALUE}"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False
        )
        logging.info("Camera exposure/gain locked (v4l2-ctl).")
    except FileNotFoundError:
        logging.warning("v4l2-ctl not found; skipping exposure lock.")

# ----------------------------
# Gate (servos)
# ----------------------------

class Gate:
    def __init__(self):
        i2c = busio.I2C(SCL, SDA)
        self.pwm = PCA9685(i2c)
        self.pwm.frequency = FREQ_HZ
        self.left = servo.Servo(self.pwm.channels[0], min_pulse=MIN_US, max_pulse=MAX_US)
        self.right = servo.Servo(self.pwm.channels[1], min_pulse=MIN_US, max_pulse=MAX_US)

        self.left_angle = clamp_angle(LEFT_CLOSED)
        self.right_angle = clamp_angle(RIGHT_CLOSED)
        self.apply()

        self.left_open_target, self.right_open_target = open_targets_from_closed()
        logging.info(f"Closed pose L={self.left_angle:.1f}, R={self.right_angle:.1f}")
        logging.info(f"Open targets L={self.left_open_target:.1f}, R={self.right_open_target:.1f}")

    def apply(self):
        self.left.angle = self.left_angle
        self.right.angle = self.right_angle

    def set_angles(self, l, r):
        self.left_angle = clamp_angle(l)
        self.right_angle = clamp_angle(r)
        self.apply()

    def force_close(self):
        self.set_angles(LEFT_CLOSED, RIGHT_CLOSED)

# ----------------------------
# Frame grabber (drop-late frames)
# ----------------------------

class FrameGrabber:
    def __init__(self, cap: cv2.VideoCapture, drop_late: bool = True):
        self.cap = cap
        self.drop_late = drop_late
        self.lock = threading.Lock()
        self.frame = None
        self.stopped = False
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while not self.stopped:
            if self.drop_late:
                # Grab then retrieve; faster path and easy to skip queued frames
                ok = self.cap.grab()
                if not ok:
                    time.sleep(0.005)
                    continue
                ok, f = self.cap.retrieve()
            else:
                ok, f = self.cap.read()
            if ok:
                with self.lock:
                    self.frame = f

    def read(self):
        with self.lock:
            if self.frame is None:
                return None
            return self.frame.copy()

    def stop(self):
        self.stopped = True
        self.thread.join(timeout=1.0)

# ----------------------------
# OCR (fast + progressive fallback)
# ----------------------------

_CANON_TABLE = str.maketrans({'0':'O', '1':'I', '2':'Z', '5':'S', '8':'B'})
def canonicalize(s: str) -> str:
    return re.sub(r"[^A-Z0-9]", "", s.upper()).translate(_CANON_TABLE)

def levenshtein(a: str, b: str) -> int:
    if a == b: return 0
    if len(a) < len(b): a, b = b, a
    prev = list(range(len(b)+1))
    for i, ca in enumerate(a, 1):
        cur = [i]
        for j, cb in enumerate(b, 1):
            ins = prev[j] + 1
            dele = cur[j-1] + 1
            sub = prev[j-1] + (ca != cb)
            cur.append(min(ins, dele, sub))
        prev = cur
    return prev[-1]

ALLOWED_PLATES_CAN = {canonicalize(p) for p in ALLOWED_PLATES}

def fuzzy_allowed(txt: str, max_dist: int = FUZZY_MAX_DIST):
    if not txt:
        return False, None, None
    can = canonicalize(txt)
    best = (False, None, None)
    for p in ALLOWED_PLATES_CAN:
        d = levenshtein(can, p)
        if d <= max_dist or (can and (can in p or p in can)):
            return True, p, d
        if best[2] is None or d < best[2]:
            best = (False, p, d)
    return best

class OCR:
    def __init__(self, allow_alphanumeric: bool = True, oem_pref: int = OEM_PREF, psm_pref: int = PSM_PREF):
        self.allow_alphanumeric = allow_alphanumeric
        self.oem_pref = oem_pref
        self.psm_pref = psm_pref
        self._use_tesserocr = False
        self._api = None
        self.last_info = "n/a"
        self._init_tesserocr()

        # Build combo lists
        self._cli_combos = []
        for inv in (0, 1):
            for oem in (self.oem_pref, 1, 3, 0, 2):
                for psm in (self.psm_pref, 7, 8, 6, 13):
                    tup = (inv, oem, psm)
                    if tup not in self._cli_combos:
                        self._cli_combos.append(tup)
        self._cli_idx = 0
        self._cli_last_success = None

        self._tess_combos = []
        for inv in (0, 1):
            for psm in (self.psm_pref, 7, 8, 6, 13):
                tup = (inv, psm)
                if tup not in self._tess_combos:
                    self._tess_combos.append(tup)
        self._tess_idx = 0
        self._tess_last_success = None

    def _init_tesserocr(self):
        try:
            import tesserocr
            from PIL import Image  # noqa: F401
            self.tesserocr = tesserocr
            self._api = tesserocr.PyTessBaseAPI(oem=self.oem_pref, psm=self.psm_pref, lang='eng')
            self._use_tesserocr = True
            self.last_info = "tesserocr ready"
            logging.info("Using libtesseract via tesserocr.")
        except Exception as e:
            self._use_tesserocr = False
            self._api = None
            logging.info(f"tesserocr not available ({e}); using tesseract CLI.")

    def close(self):
        if self._api is not None:
            try: self._api.End()
            except Exception: pass
            self._api = None

    def _clean(self, raw: str) -> str:
        if self.allow_alphanumeric:
            return re.sub(r"[^A-Z0-9]", "", raw.upper())
        return re.sub(r"[^0-9]", "", raw)

    def _ensure_size(self, img_bw):
        h = img_bw.shape[0]
        if h < UPSCALE_HEIGHT:
            scale = UPSCALE_HEIGHT / float(h)
            img_bw = cv2.resize(img_bw, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        return img_bw

    def read(self, img_bw) -> str:
        img_bw = self._ensure_size(img_bw)
        if self._use_tesserocr and self._api is not None:
            out = self._read_tesserocr_step(img_bw, OCR_MAX_COMBOS_PER_FRAME)
            if out: return out
        return self._read_cli_step(img_bw, OCR_MAX_COMBOS_PER_FRAME)

    # ---- tesserocr progressive step
    def _read_tesserocr_step(self, img_bw, steps: int):
        from PIL import Image
        whitelist = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789" if self.allow_alphanumeric else "0123456789"

        def run(inv, psm):
            try:
                self._api.SetVariable("tessedit_char_whitelist", whitelist)
                self._api.SetPageSegMode(psm)
                pil_img = Image.fromarray(img_bw if inv == 0 else cv2.bitwise_not(img_bw))
                self._api.SetImage(pil_img)
                raw = self._api.GetUTF8Text() or ""
                cleaned = self._clean(raw)
                self.last_info = f"tess psm={psm} inv={inv}"
                return cleaned
            except Exception as e:
                self.last_info = f"tess err={e}"
                return ""

        # Try last success first
        if self._tess_last_success is not None:
            inv, psm = self._tess_last_success
            out = run(inv, psm)
            if out:
                return out

        # Try a few combos this frame
        for _ in range(steps):
            inv, psm = self._tess_combos[self._tess_idx]
            self._tess_idx = (self._tess_idx + 1) % len(self._tess_combos)
            out = run(inv, psm)
            if out:
                self._tess_last_success = (inv, psm)
                return out
        return ""

    # ---- CLI progressive step
    def _read_cli_step(self, img_bw, steps: int):
        whitelist = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789" if self.allow_alphanumeric else "0123456789"
        binary = TESSERACT_BIN if os.path.exists(TESSERACT_BIN) else "tesseract"

        def run(inv, oem, psm):
            arr = img_bw if inv == 0 else cv2.bitwise_not(img_bw)
            ok, png = cv2.imencode(".png", arr)
            if not ok:
                return ""
            cmd = [
                binary, "stdin", "stdout",
                "--oem", str(oem),
                "--psm", str(psm),
                "-l", "eng",
                "-c", f"tessedit_char_whitelist={whitelist}",
            ]
            out = subprocess.run(cmd, input=png.tobytes(),
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
            raw = out.stdout.decode("utf-8", errors="ignore").strip()
            cleaned = self._clean(raw)
            self.last_info = f"cli oem={oem} psm={psm} inv={inv}"
            return cleaned

        # Try last success first
        if self._cli_last_success is not None:
            inv, oem, psm = self._cli_last_success
            out = run(inv, oem, psm)
            if out:
                return out

        # Try a few combos this frame
        for _ in range(steps):
            inv, oem, psm = self._cli_combos[self._cli_idx]
            self._cli_idx = (self._cli_idx + 1) % len(self._cli_combos)
            out = run(inv, oem, psm)
            if out:
                self._cli_last_success = (inv, oem, psm)
                return out
        return ""

# ----------------------------
# Vision / OCR preprocessing
# ----------------------------

def preprocess(gray):
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, bw = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return bw

def open_camera():
    # You can also try cv2.CAP_GSTREAMER with appsink drop=true if desired.
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2) if USE_V4L2 else cv2.VideoCapture(CAM_INDEX)
    if not cap.isOpened():
        raise RuntimeError("Camera not found / cannot be opened.")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, TARGET_FPS)
    # shrink internal buffer if supported (best-effort)
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass
    if LOCK_CAMERA_EXPOSURE:
        safe_run_v4l2ctl(f"/dev/video{CAM_INDEX}")
    return cap

# ----------------------------
# Cycle state machine (threaded, time-based, smooth)
# ----------------------------

class Cycle:
    def __init__(self, gate: Gate):
        self.gate = gate
        self.state = "READY"
        self.lock = threading.Lock()
        self.phase_start = 0.0
        self.phase_duration = 0.0
        self.start_L = gate.left_angle
        self.start_R = gate.right_angle
        self.tgt_L = gate.left_open_target
        self.tgt_R = gate.right_open_target
        self.hold_until = 0.0
        self.stop_evt = threading.Event()
        self.worker = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker.start()

    def start(self, reason="PLATE"):
        with self.lock:
            if self.state != "READY":
                logging.info(f"Ignored start({reason}) because state={self.state}")
                return False
            logging.info(f"Cycle START by {reason}")
            self.state = "OPENING"
            self.phase_start = time.perf_counter()
            self.phase_duration = OPEN_DURATION_S
            self.start_L, self.start_R = self.gate.left_angle, self.gate.right_angle
            self.tgt_L, self.tgt_R = self.gate.left_open_target, self.gate.right_open_target
        return True

    def cancel_and_close(self):
        with self.lock:
            logging.info("Cycle CANCEL -> force CLOSE")
            self.gate.force_close()
            self.state = "READY"

    def _worker_loop(self):
        while not self.stop_evt.is_set():
            with self.lock:
                st = self.state
                now = time.perf_counter()
                if st == "OPENING":
                    t = (now - self.phase_start) / max(1e-6, self.phase_duration)
                    if t >= 1.0:
                        self.gate.set_angles(self.tgt_L, self.tgt_R)
                        self.state = "HOLDING"
                        self.hold_until = now + HOLD_OPEN_SECONDS
                    else:
                        e = ease_in_out_cosine(t)
                        L = self.start_L + (self.tgt_L - self.start_L) * e
                        R = self.start_R + (self.tgt_R - self.start_R) * e
                        self.gate.set_angles(L, R)
                elif st == "HOLDING":
                    if now >= self.hold_until:
                        self.state = "CLOSING"
                        self.phase_start = now
                        self.phase_duration = CLOSE_DURATION_S
                        self.start_L, self.start_R = self.gate.left_angle, self.gate.right_angle
                        self.tgt_L, self.tgt_R = LEFT_CLOSED, RIGHT_CLOSED
                elif st == "CLOSING":
                    t = (now - self.phase_start) / max(1e-6, self.phase_duration)
                    if t >= 1.0:
                        self.gate.set_angles(self.tgt_L, self.tgt_R)
                        self.state = "READY"
                    else:
                        e = ease_in_out_cosine(t)
                        L = self.start_L + (self.tgt_L - self.start_L) * e
                        R = self.start_R + (self.tgt_R - self.start_R) * e
                        self.gate.set_angles(L, R)
            time.sleep(APPLY_INTERVAL_S)

    def stop(self):
        self.stop_evt.set()
        self.worker.join(timeout=1.0)

# ----------------------------
# Main
# ----------------------------

def main():
    cap = open_camera()
    grabber = FrameGrabber(cap, drop_late=DROP_LATE_FRAMES)

    gate = Gate()
    cycle = Cycle(gate)
    ocr = OCR(allow_alphanumeric=ALLOW_ALPHANUMERIC, oem_pref=OEM_PREF, psm_pref=PSM_PREF)

    match_buffer = collections.deque(maxlen=MATCH_FRAMES)

    try:
        while True:
            frame = grabber.read()
            if frame is None:
                # No frame yet; let UI breathe
                cv2.waitKey(1)
                continue

            h, w = frame.shape[:2]
            x, y, wf, hf = PLATE_ROI
            x0, y0 = int(x * w), int(y * h)
            x1, y1 = int((x + wf) * w), int((y + hf) * h)
            x0 = max(0, min(w - 1, x0)); x1 = max(1, min(w, x1))
            y0 = max(0, min(h - 1, y0)); y1 = max(1, min(h, y1))
            if x1 <= x0 or y1 <= y0:
                # Fallback center box
                cx, cy = w // 2, h // 2
                rw, rh = w // 2, h // 2
                x0, y0, x1, y1 = cx - rw//2, cy - rh//2, cx + rw//2, cy + rh//2

            roi = frame[y0:y1, x0:x1]

            if cycle.state == "READY":
                gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                bw = preprocess(gray)

                t0 = time.perf_counter()
                plate_text = ocr.read(bw)  # progressive, limited work per frame
                dt_ms = (time.perf_counter() - t0) * 1000.0

                is_valid_len = len(plate_text) >= MIN_PLATE_LEN
                okish, target, dist = fuzzy_allowed(plate_text)
                match = is_valid_len and okish

                match_buffer.append(match)
                matched_now = sum(match_buffer) == MATCH_FRAMES

                if matched_now:
                    cycle.start("PLATE")
                    match_buffer.clear()

                # UI overlays
                box_color = (0, 255, 0) if match else (0, 0, 255)
                cv2.rectangle(frame, (x0, y0), (x1, y1), box_color, 2)
                status = f"Read: {plate_text or '-'}  "
                if target is not None:
                    status += f"→ {target} (d={dist})  "
                status += f"OCR: {dt_ms:.1f}ms [{ocr.last_info}]  State: {cycle.state}"
                cv2.putText(frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)

                cv2.imshow("ROI - binarized", bw)
            else:
                cv2.rectangle(frame, (x0, y0), (x1, y1), (0, 128, 255), 2)
                cv2.putText(
                    frame, f"Read: (busy)  State: {cycle.state}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2
                )

            cv2.imshow("Vision Gate", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == ord('o'):
                cycle.start("MANUAL")
            elif key == ord('c'):
                cycle.cancel_and_close()

    finally:
        try: cycle.stop()
        except Exception: pass
        try: ocr.close()
        except Exception: pass
        try: gate.force_close()
        except Exception: pass
        try: grabber.stop()
        except Exception: pass
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()

