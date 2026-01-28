# =========================
# ROBO_OS v3.1 — FULL MIX (NO PRINT)
# =========================

import time, os, tempfile, subprocess
import numpy as np
import cv2
import sounddevice as sd
import soundfile as sf
import Jetson.GPIO as GPIO
import pyrealsense2 as rs
import ollama
from gradio_client import Client
from adafruit_pca9685 import PCA9685
import busio
from board import SCL, SDA

# =========================
# SETTINGS
# =========================
GPIO.setmode(GPIO.BOARD)
TOUCH_PIN = 7
GPIO.setup(TOUCH_PIN, GPIO.IN)

# =========================
# SERVOS (PCA9685)
# =========================
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

def set_servo(channel, angle):
    pulse = int(150 + (angle / 180) * 450)
    pca.channels[channel].duty_cycle = pulse << 4

# =========================
# BODY PARTS
# =========================
class Leg:
    def __init__(self, hip, knee, ankle):
        self.hip = hip
        self.knee = knee
        self.ankle = ankle
    def step_forward(self):
        set_servo(self.hip, 60)
        set_servo(self.knee, 90)
        set_servo(self.ankle, 70)
        time.sleep(0.2)

class Arm:
    def __init__(self, shoulder, elbow, wrist):
        self.shoulder = shoulder
        self.elbow = elbow
        self.wrist = wrist
    def wave(self):
        set_servo(self.shoulder, 120)
        set_servo(self.elbow, 60)
        set_servo(self.wrist, 90)

class Hand:
    def __init__(self, fingers):
        self.fingers = fingers
    def open(self):
        for f in self.fingers: set_servo(f, 10)
    def fist(self):
        for f in self.fingers: set_servo(f, 90)

class Face:
    def __init__(self, brow_left, brow_right, eye_left, eye_right, mouth):
        self.brow_left = brow_left
        self.brow_right = brow_right
        self.eye_left = eye_left
        self.eye_right = eye_right
        self.mouth = mouth
    def smile(self):
        set_servo(self.mouth, 120)
    def blink(self):
        set_servo(self.eye_left, 30)
        set_servo(self.eye_right, 30)
        time.sleep(0.1)
        set_servo(self.eye_left, 90)
        set_servo(self.eye_right, 90)

# =========================
# CREATE BODY
# =========================
left_leg = Leg(0,1,2)
right_leg = Leg(3,4,5)
left_arm = Arm(6,7,8)
right_arm = Arm(9,10,11)
right_hand = Hand([12,13,14,15,16])
face = Face(17,18,19,20,21)

# =========================
# REALSENSE + OPENCV
# =========================
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

def see():
    frames = pipeline.wait_for_frames()
    color = frames.get_color_frame()
    if not color: return None
    return np.asanyarray(color.get_data())

# =========================
# COQUI STT
# =========================
def listen():
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        path = f.name
    duration = 4
    audio = sd.rec(int(duration*16000), samplerate=16000, channels=1)
    sd.wait()
    sf.write(path, audio, 16000)
    try:
        text = subprocess.check_output(["stt","--audio",path], text=True).strip()
    except:
        text = None
    os.remove(path)
    return text

# =========================
# BRAIN
# =========================
brain_gradio = Client("http://localhost:7861")

def think_gradio(text):
    try:
        return brain_gradio.predict(text, api_name="/predict")
    except:
        return None

def think(text):
    try:
        r = ollama.chat(
            model="phi3",
            messages=[
                {"role":"system","content":"Ты робот с телом, зрением и слухом."},
                {"role":"user","content":text}
            ]
        )
        return r["message"]["content"]
    except:
        return None

# =========================
# REFLEX
# =========================
def reflex():
    if GPIO.input(TOUCH_PIN):
        return "Больно. Я остановился."

# =========================
# COMMAND PARSER
# =========================
def act(command):
    t = command.lower()

    if "иди" in t:
        left_leg.step_forward()
        right_leg.step_forward()
        return "Я иду."

    if "стоп" in t:
        return "Я остановился."

    if "помаши" in t:
        right_arm.wave()
        return "Привет!"

    if "открой руку" in t:
        right_hand.open()
        return "Рука открыта."

    if "сжать руку" in t:
        right_hand.fist()
        return "Рука сжата."

    if "улыбнись" in t:
        face.smile()
        return "😊"

    if "моргни" in t:
        face.blink()
        return "😉"

    answer = think(command)
    if not answer:
        answer = think_gradio(command)
    return answer

# =========================
# MAIN LOOP
# =========================
def main():
    try:
        while True:
            frame = see()
            if frame is not None:
                cv2.imshow("vision", frame)
                cv2.waitKey(1)

            r = reflex()
            if r:
                time.sleep(0.3)
                continue

            command = listen()
            if not command: continue

            response = act(command)
            time.sleep(0.1)
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        GPIO.cleanup()

# =========================
# START
# =========================
if __name__ == "__main__":
    main()
