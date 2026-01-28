# =========================
# ROBO_OS v3 — MIX CORE
# =========================

__robo__ = "robo_OS__v3"

import time, os, tempfile
import ollama
import Jetson.GPIO as GPIO
import cv2
import numpy as np
import pyrealsense2 as rs
import sounddevice as sd
import soundfile as sf
from gradio_client import Client
import subprocess

# =========================
# GPIO (ТЕЛО)
# =========================
GPIO.setmode(GPIO.BOARD)

TOUCH_PIN = 7
MOTOR_L = 16
MOTOR_R = 18

GPIO.setup(TOUCH_PIN, GPIO.IN)
GPIO.setup([MOTOR_L, MOTOR_R], GPIO.OUT)

def move_forward():
    GPIO.output(MOTOR_L, 1)
    GPIO.output(MOTOR_R, 1)

def move_stop():
    GPIO.output(MOTOR_L, 0)
    GPIO.output(MOTOR_R, 0)

# =========================
# REALSENSE + OPENCV (ЗРЕНИЕ)
# =========================
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

def see():
    frames = pipeline.wait_for_frames()
    color = frames.get_color_frame()
    if not color:
        return None
    return np.asanyarray(color.get_data())

# =========================
# COQUI STT (СЛУХ)
# =========================
def listen():
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        path = f.name

    duration = 4
    audio = sd.rec(int(duration * 16000), samplerate=16000, channels=1)
    sd.wait()
    sf.write(path, audio, 16000)

    try:
        text = subprocess.check_output(
            ["stt", "--audio", path],
            text=True
        ).strip()
    except:
        text = None

    os.remove(path)
    return text

# =========================
# GRADIO CLIENT (OPTIONAL BRAIN / TOOLS)
# =========================
brain_gradio = Client("http://localhost:7861")  # если есть

def think_gradio(text):
    try:
        return brain_gradio.predict(text, api_name="/predict")
    except:
        return None

# =========================
# OLLAMA (ОСНОВНОЙ МОЗГ)
# =========================
def think(text):
    r = ollama.chat(
        model="phi3",
        messages=[
            {"role": "system", "content": "Ты робот с телом, зрением и слухом."},
            {"role": "user", "content": text}
        ]
    )
    return r["message"]["content"]

# =========================
# REFLEX SYSTEM
# =========================
def reflex():
    if GPIO.input(TOUCH_PIN):
        move_stop()
        return "Больно. Я остановился."
    return None

# =========================
# COMMAND PARSER
# =========================
def act(command):
    t = command.lower()

    if "иди" in t:
        move_forward()
        return "Я иду."

    if "стоп" in t:
        move_stop()
        return "Я остановился."

    # сначала локальный мозг
    answer = think(command)

    # если нужен внешний интеллект
    if not answer:
        answer = think_gradio(command)

    return answer

# =========================
# MAIN LOOP
# =========================
def main():
    print(__robo__, "online")

    while True:
        # зрение (можно анализировать)
        frame = see()
        if frame is not None:
            cv2.imshow("vision", frame)
            cv2.waitKey(1)

        # рефлексы — приоритет
        r = reflex()
        if r:
            print("REFLEX:", r)
            time.sleep(0.3)
            continue

        # слух
        command = listen()
        if not command:
            continue

        print("HEARD:", command)

        # действие + мышление
        response = act(command)
        print("ROBO:", response)

        time.sleep(0.1)

# =========================
# START
# =========================
try:
    main()
finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    GPIO.cleanup()
