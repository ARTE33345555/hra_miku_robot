import time
import os
import tempfile
import subprocess
import socket
import threading
import queue
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

# ==============================================================================
# 1. СЕКРЕТИ, ПЕРИФЕРІЯ ТА ІНІЦІАЛІЗАЦІЯ
# ==============================================================================
KEY = b'PASTE_YOUR_SECRET_KEY_HERE'
fernet = Fernet(KEY)
HOST, PORT = "0.0.0.0", 5050
ROBOT_NAME = "артем"

# Налаштування GPIO
GPIO.setmode(GPIO.BOARD)
L_HIP, R_HIP, L_KNEE, R_KNEE = 24, 32, 26, 33
TOUCH_PIN = 7
pins = [L_HIP, R_HIP, L_KNEE, R_KNEE]

GPIO.setup(pins, GPIO.OUT)
GPIO.setup(TOUCH_PIN, GPIO.IN)

pwms = {p: GPIO.PWM(p, 50) for p in pins}
for p in pwms.values(): 
    p.start(0)

# Ініціалізація IMU (MPU6050)
try:
    imu = mpu6050(0x68)
    HAS_IMU = True
except Exception:
    HAS_IMU = False

# Ініціалізація TTS Клієнта та глобальних прапорців
miku_client = Client("John6666/mikuTTS")
last_activity = time.time()
is_running = True

# Потокобезпечна черга для текстових команд (Голос + Мережа)
command_queue = queue.Queue()

# ==============================================================================
# 2. КЕРУВАННЯ РУХОМ ТА PID СТАБІЛІЗАЦІЯ
# ==============================================================================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.last_err = 0
        self.integral = 0
        
    def compute(self, error):
        self.integral += error
        derivative = error - self.last_err
        self.last_err = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

balancer = PID(1.6, 0.02, 0.45)

def set_legs(hip, knee):
    try:
        pwms[L_HIP].ChangeDutyCycle(2 + hip / 18)
        pwms[L_KNEE].ChangeDutyCycle(2 + (90 + knee) / 18)
        pwms[R_HIP].ChangeDutyCycle(2 + (180 - hip) / 18)
        pwms[R_KNEE].ChangeDutyCycle(2 + (90 - knee) / 18)
    except Exception:
        pass

def stability_thread():
    """Фоновий потік фізичного балансування робота."""
    while is_running:
        if HAS_IMU:
            try:
                tilt = imu.get_accel_data()['x']
                corr = balancer.compute(tilt)
                target_h = np.clip(90 + corr, 70, 110)
                set_legs(target_h, corr * 0.75)
            except Exception:
                pass
        time.sleep(0.015)

# ==============================================================================
# 3. МОВНІ ТЕХНОЛОГІЇ (STT / TTS)
# ==============================================================================
def get_voice(lang):
    # Додано український голос за замовчуванням (через Ostap або Polina)
    voices = {
        "UA": "uk-UA-OstapNeural-Male", 
        "RU": "ru-RU-SvetlanaNeural-Female",
        "EN": "en-US-AnaNeural-Female",
        "JA": "ja-JP-NanamiNeural-Female"
    }
    return voices.get(lang, "uk-UA-OstapNeural-Male")

def speak(text):
    """Синтез мовлення через віддалений API Miku TTS."""
    try:
        lang, content = "UA", text
        if "]" in text:
            tag, content = text.split("]", 1)
            lang = tag.replace("[", "").strip().upper()
            
        voice = get_voice(lang)
        res = miku_client.predict(
            model_name="1a_miku_default_rvc_(aple)",
            tts_text=content.strip(),
            tts_voice=voice,
            speed=0,
            f0_up_key=6,
            f0_method="rmvpe",
            index_rate=1,
            protect=0.33,
            api_name="/tts"
        )
        os.system(f"aplay {res} > /dev/null 2>&1")
    except Exception:
        pass

def listen():
    """Запис звуку та локальне розпізнавання мовлення (STT)."""
    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as f:
        path = f.name
    try:
        audio = sd.rec(int(3.8 * 16000), samplerate=16000, channels=1)
        sd.wait()
        sf.write(path, audio, 16000)
        t = subprocess.check_output(["stt", "--audio", path], text=True).strip()
    except Exception:
        t = None
    finally:
        if os.path.exists(path):
            os.remove(path)
    return t

def voice_listening_thread():
    """Фоновий потік постійного прослуховування ефіру."""
    while is_running:
        cmd = listen()
        if cmd:
            command_queue.put(cmd)
        time.sleep(0.1)

# ==============================================================================
# 4. ОБРОБКА КОМАНД ТА ШІ (OLLAMA)
# ==============================================================================
def mark_activity():
    global last_activity
    last_activity = time.time()

def act(command):
    """Парсинг команд та інференс LLM моделі."""
    if not command:
        return
    mark_activity()
    t = command.lower()
    
    if ROBOT_NAME in t:
        speak("[UA] Я тут.")
        return
    if "іди" in t or "вперед" in t:
        set_legs(105, 15)
        speak("[UA] Починаю рух.")
        return
    if "стій" in t or "встань" in t or "зупинись" in t:
        set_legs(90, 0)
        speak("[UA] Зупинилась.")
        return
        
    prompt = f"Role: Humanoid Robot Artem. Task: Detect language and respond with tags like [UA],[EN],[JA]. User: {command}"
    try:
        r = ollama.chat(model="phi3", messages=[{"role": "user", "content": prompt}])
        speak(r["message"]["content"])
    except Exception:
        pass

def command_worker_thread():
    """Ізольований обробник черги завдань."""
    while is_running:
        try:
            cmd = command_queue.get(timeout=0.5)
            act(cmd)
            command_queue.task_done()
        except queue.Empty:
            continue

# ==============================================================================
# 5. ЕНЕРГОСПИТАННЯ ТА СИСТЕМА ЖИВЛЕННЯ
# ==============================================================================
def get_battery_percent():
    try:
        with open("/sys/class/power_supply/BAT0/capacity") as f:
            return int(f.read().strip())
    except Exception:
        return 100

def hibernate():
    global is_running
    speak("[UA] Я засинаю.")
    set_legs(90, 0)
    time.sleep(1)
    is_running = False
    os.system("systemctl suspend")

def go_to_dock():
    speak("[UA] Заряд майже вичерпано. Йду на док-станцію.")
    for _ in range(20):
        set_legs(105, 15)
        time.sleep(0.2)
    set_legs(80, -10)
    speak("[UA] Я на місці. Переходжу в режим сну.")
    hibernate()

# ==============================================================================
# 6. МЕРЕЖЕВИЙ TCP ІНТЕРФЕЙС
# ==============================================================================
def tcp_handler():
    """Фоновий сервер для прийому зашифрованих команд керування."""
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    server.settimeout(1.0)
    
    while is_running:
        try:
            conn, _ = server.accept()
            data = conn.recv(4096)
            if data:
                cmd = fernet.decrypt(data).decode()
                command_queue.put(cmd)
            conn.close()
        except socket.timeout:
            continue
        except Exception:
            pass

# ==============================================================================
# 7. ГОЛОВНИЙ ПОТІК: ЗОР (REALSENSE) ТА ДАТЧИКИ
# ==============================================================================
def main():
    global is_running
    
    # Запуск паралельної архітектури потоків
    threading.Thread(target=stability_thread, daemon=True).start()
    threading.Thread(target=tcp_handler, daemon=True).start()
    threading.Thread(target=voice_listening_thread, daemon=True).start()
    threading.Thread(target=command_worker_thread, daemon=True).start()

    # Запуск камери Intel RealSense
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    try:
        pipe.start(cfg)
    except Exception as e:
        print(f"Критична помилка RealSense: {e}")
        is_running = False
        GPIO.cleanup()
        return

    try:
        while is_running:
            # Контроль рівня заряду батареї
            if get_battery_percent() <= 10:
                go_to_dock()
                break

            # Читання та рендеринг кадру (Суворо Main Thread)
            try:
                frames = pipe.wait_for_frames(timeout_ms=80)
                frame = frames.get_color_frame()
                if frame:
                    img = np.asanyarray(frame.get_data())
                    cv2.imshow("HRA_V5", img)
            except Exception:
                pass

            # Обробка закриття графічного вікна після натискання клавіші 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'): 
                break

            # Опитування апаратного датчика дотику
            try:
                if GPIO.input(TOUCH_PIN) == GPIO.LOW:
                    speak("[UA] Будь ласка, обережніше.")
                    mark_activity()
                    time.sleep(0.4)
                    continue
            except Exception:
                pass

            # Перевірка таймера сну (5 хвилин бездіяльності)
            if time.time() - last_activity > 300:
                hibernate()
                break

    finally:
        is_running = False
        try:
            pipe.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        GPIO.cleanup()

if __name__ == "__main__":
    main()
