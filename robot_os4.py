# =========================
# ROBOT_OS v4 HRA — Human Robot Artem
# =========================

__robo__ = "robot_os_v4_hra"

import time, os, tempfile, subprocess, socket, threading
import Jetson.GPIO as GPIO
import cv2
import numpy as np
import pyrealsense2 as rs
import sounddevice as sd
import soundfile as sf
from cryptography.fernet import Fernet
import ollama
from gradio_client import Client

# =========================
# ЗАЩИЩЁННЫЙ КЛЮЧ ДЛЯ TCP
# =========================
KEY = b'PASTE_YOUR_SECRET_KEY_HERE'
fernet = Fernet(KEY)
HOST, PORT = "0.0.0.0", 5050

# =========================
# GPIO (СЕРВОПРИВОДЫ)
# =========================
GPIO.setmode(GPIO.BOARD)
TOUCH_PIN = 7
SERVO_PIN = 22        # сустав
HIP_SERVO_PIN = 24    # таз/бедро
ARM_SERVO_PIN = 26    # рука/локоть

GPIO.setup(TOUCH_PIN, GPIO.IN)
GPIO.setup([SERVO_PIN, HIP_SERVO_PIN, ARM_SERVO_PIN], GPIO.OUT)

servo = GPIO.PWM(SERVO_PIN, 50); servo.start(0)
hip_servo = GPIO.PWM(HIP_SERVO_PIN, 50); hip_servo.start(0)
arm_servo = GPIO.PWM(ARM_SERVO_PIN, 50); arm_servo.start(0)

def set_servo_angle(angle):
    servo.ChangeDutyCycle(2 + angle/18)
    time.sleep(0.3); servo.ChangeDutyCycle(0)
def set_hip_angle(angle):
    hip_servo.ChangeDutyCycle(2 + angle/18)
    time.sleep(0.3); hip_servo.ChangeDutyCycle(0)
def set_arm_angle(angle):
    arm_servo.ChangeDutyCycle(2 + angle/18)
    time.sleep(0.3); arm_servo.ChangeDutyCycle(0)

# =========================
# ЗРЕНИЕ (Realsense + OpenCV)
# =========================
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)
pipeline.start(config)

def see():
    frames = pipeline.wait_for_frames()
    color = frames.get_color_frame()
    if not color: return None
    return np.asanyarray(color.get_data())

# =========================
# СЛУХ (Coqui STT)
# =========================
def listen():
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        path = f.name
    audio = sd.rec(int(4*16000), samplerate=16000, channels=1)
    sd.wait(); sf.write(path, audio, 16000)
    try: text = subprocess.check_output(["stt","--audio",path], text=True).strip()
    except: text=None
    os.remove(path)
    return text

# =========================
# TTS (Miku / СВОЙ ГОЛОС)
# =========================
tts_client = Client("John6666/mikuTTS")
def speak(text):
    result = tts_client.predict(
        model_name="1a_miku_default_rvc_(aple)",
        speed=0,
        tts_text=text,
        tts_voice="custom_voice",
        f0_up_key=3,
        f0_method="rmvpe",
        index_rate=1,
        protect=0.3,
        api_name="/tts"
    )
    sf.write("speech.wav", result, 44100)
    os.system("aplay speech.wav")
    return f"Я сказала: {text}"

# =========================
# МОЗГ (Ollama)
# =========================
def think(text):
    r = ollama.chat(model="phi3", messages=[
        {"role":"system","content":"Ты Humanoidroid с руками, ногами, слухом, зрением и голосом."},
        {"role":"user","content":text}
    ])
    return r["message"]["content"]

# =========================
# REFLEX
# =========================
def reflex():
    if GPIO.input(TOUCH_PIN):
        set_hip_angle(0); set_arm_angle(0); set_servo_angle(0)
        return "Больно! Я остановился."
    return None

# =========================
# COMMAND PARSER
# =========================
def act(command):
    t = command.lower()
    if "иди" in t: set_hip_angle(45); return speak("Я иду.")
    if "стоп" in t: set_hip_angle(0); return speak("Я остановился.")
    if "согнуть руку" in t: set_arm_angle(90); return speak("Я согнула руку.")
    if "разогнуть руку" in t: set_arm_angle(0); return speak("Я разогнула руку.")
    if "сказать" in t: return speak(command.replace("сказать","").strip())
    # если нет локальной команды
    answer = think(command)
    if not answer: return speak("Я не поняла.")
    return answer

# =========================
# SECURE TCP SERVER
# =========================
def start_tcp_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT)); server.listen(1)
    print("🔒 Secure Robo TCP Server online")
    conn, addr = server.accept()
    print("Connected:", addr)
    while True:
        data = conn.recv(4096)
        if not data: break
        decrypted = fernet.decrypt(data).decode()
        print("CMD TCP:", decrypted)
        response = act(decrypted)
        encrypted = fernet.encrypt(response.encode())
        conn.send(encrypted)

# =========================
# MAIN LOOP
# =========================
def main():
    tcp_thread = threading.Thread(target=start_tcp_server, daemon=True)
    tcp_thread.start()
    print(__robo__,"online")
    while True:
        frame = see()
        if frame is not None:
            cv2.imshow("vision", frame)
            cv2.waitKey(1)
        r = reflex()
        if r: print("REFLEX:",r); time.sleep(0.3); continue
        command = listen()
        if not command: continue
        print("HEARD:",command)
        response = act(command)
        print("ROBO:",response)
        time.sleep(0.1)

# =========================
# START
# =========================
try:
    main()
finally:
    pipeline.stop(); cv2.destroyAllWindows()
    servo.stop(); hip_servo.stop(); arm_servo.stop()
    GPIO.cleanup()
