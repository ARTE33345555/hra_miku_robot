import time, os, tempfile, subprocess, socket, threading
import Jetson.GPIO as GPIO
import cv2
import numpy as np
import pyrealsense2 as rs
import sounddevice as sd
import soundfile as sf
from cryptography.fernet import Fernet
import ollama
from mpu6050 import mpu6050
from gradio_client import Client

# =========================
# 1. СЕКРЕТЫ И ПЕРИФЕРИЯ
# =========================
KEY = b'PASTE_YOUR_SECRET_KEY_HERE'
fernet = Fernet(KEY)
HOST, PORT = "0.0.0.0", 5050

GPIO.setmode(GPIO.BOARD)
L_HIP, R_HIP, L_KNEE, R_KNEE = 24, 32, 26, 33
TOUCH_PIN = 7
pins = [L_HIP, R_HIP, L_KNEE, R_KNEE]

GPIO.setup(pins, GPIO.OUT)
GPIO.setup(TOUCH_PIN, GPIO.IN)

pwms = {p: GPIO.PWM(p, 50) for p in pins}
for p in pwms.values(): p.start(0)

try:
    imu = mpu6050(0x68)
    HAS_IMU = True
except:
    HAS_IMU = False

miku_client = Client("John6666/mikuTTS")

# =========================
# 2. СТАБИЛИЗАЦИЯ И ФИЗИКА
# =========================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.last_err = 0
        self.integral = 0
    def compute(self, error):
        self.integral += error
        derivative = error - self.last_err
        self.last_err = error
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

balancer = PID(1.6, 0.02, 0.45)

def set_legs(hip, knee):
    pwms[L_HIP].ChangeDutyCycle(2 + hip/18)
    pwms[L_KNEE].ChangeDutyCycle(2 + (90 + knee)/18)
    pwms[R_HIP].ChangeDutyCycle(2 + (180 - hip)/18)
    pwms[R_KNEE].ChangeDutyCycle(2 + (90 - knee)/18)

def stability_thread():
    while True:
        if HAS_IMU:
            tilt = imu.get_accel_data()['x']
            corr = balancer.compute(tilt)
            # Ограничение углов для безопасности
            target_h = np.clip(90 + corr, 70, 110)
            set_legs(target_h, corr * 0.75)
        time.sleep(0.015)

# =========================
# 3. ПОЛИГЛОТ TTS И STT
# =========================
def get_voice(lang):
    v = {"RU": "ru-RU-SvetlanaNeural-Female", 
         "JA": "ja-JP-NanamiNeural-Female",
         "EN": "en-US-AnaNeural-Female"}
    return v.get(lang, "en-US-AnaNeural-Female")

def speak(raw_text):
    try:
        lang, text = "EN", raw_text
        if "]" in raw_text:
            tag, text = raw_text.split("]", 1)
            lang = tag.replace("[", "").strip().upper()
        
        voice = get_voice(lang)
        res = miku_client.predict(
            model_name="1a_miku_default_rvc_(aple)",
            speed=0, tts_text=text.strip(), tts_voice=voice,
            f0_up_key=6, f0_method="rmvpe", index_rate=1, protect=0.33,
            api_name="/tts"
        )
        os.system(f"aplay {res} > /dev/null 2>&1")
    except: pass

def listen():
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        path = f.name
    audio = sd.rec(int(3.8 * 16000), samplerate=16000, channels=1)
    sd.wait(); sf.write(path, audio, 16000)
    try:
        t = subprocess.check_output(["stt","--audio",path], text=True).strip()
    except: t = None
    if os.path.exists(path): os.remove(path)
    return t

# =========================
# 4. МОЗГ И КОМАНДЫ
# =========================
def act(command):
    if not command: return
    t = command.lower()
    
    if "иди" in t: 
        set_legs(105, 15)
        return speak("[RU] Начинаю движение.")
    if "стой" in t or "встань" in t: 
        set_legs(90, 0)
        return speak("[RU] Остановилась.")

    prompt = f"Role: Humanoid Robot Artem. Task: Detect language and respond with tags like [RU], [EN], [JA]. User: {command}"
    r = ollama.chat(model="phi3", messages=[{"role":"user","content":prompt}])
    speak(r["message"]["content"])

# =========================
# 5. СЕТЬ И ГЛАВНЫЙ ЦИКЛ
# =========================
def tcp_handler():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT)); server.listen(1)
    while True:
        try:
            conn, _ = server.accept()
            data = conn.recv(4096)
            if data:
                cmd = fernet.decrypt(data).decode()
                act(cmd)
        except: pass

def main():
    threading.Thread(target=stability_thread, daemon=True).start()
    threading.Thread(target=tcp_handler, daemon=True).start()
    
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipe.start(cfg)

    try:
        while True:
            # Зрение
            f = pipe.wait_for_frames()
            img = np.asanyarray(f.get_color_frame().get_data())
            cv2.imshow("HRA_V5", img); cv2.waitKey(1)
            
            # Рефлекс боли
            if GPIO.input(TOUCH_PIN):
                speak("[RU] Пожалуйста, осторожнее."); time.sleep(1); continue
            
            # Слух
            cmd = listen()
            if cmd: act(cmd)
    finally:
        pipe.stop(); cv2.destroyAllWindows(); GPIO.cleanup()

if __name__ == "__main__":
    main()
