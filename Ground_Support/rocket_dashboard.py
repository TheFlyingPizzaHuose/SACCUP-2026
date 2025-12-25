import os
import time
import random
import threading
import serial
import serial.tools.list_ports  # optional, for listing ports
from flask import Flask, jsonify, render_template, Response
from flask_socketio import SocketIO, emit
import socket
import struct
import serial.tools.list_ports
import math

start_time = time.time()

lock = threading.Lock() #prevents race conditions on the data variable
data = []

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__, template_folder=os.path.join(BASE_DIR, "templates"))
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="eventlet")

#creating data folder and log file
path = "data"
port = 4000

if not os.path.exists(path):
    os.makedirs(path)  # creates the folder
    print(f"Created folder: {path}")
filenames = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f))]#check files present in folder
if filenames: #if there are files in the folder at all
    filenames_no_ext = [os.path.splitext(f)[0] for f in filenames] #removes file exentions
    filenumbers = [float(x) for x in filenames_no_ext]
    log_file =  str(max(filenumbers) + 1)+ ".txt"
else:
    log_file = "0.txt"

TARGET_KEYWORD = "Teensy"   # or use VID like "16C0"
BAUD = 115200
def serial_thread():
    ser = None
    while True:
        # Check if serial is open
        if ser is None or not ser.is_open:
            port = None
            for p in serial.tools.list_ports.comports():
                if TARGET_KEYWORD in p.description or TARGET_KEYWORD in p.hwid:
                    port = p.device
                    break
                print("Teensy not found")
                time.sleep(1)
                
            if port:
                try:
                    ser = serial.Serial(port, BAUD, timeout=0.1)
                except serial.SerialException:
                    print(f"Failed to open {port}")
                    ser = None

        # Read serial if connected
        if ser and ser.is_open:
            try:
                if ser.in_waiting:
                    raw_line_string = ser.readline().decode(errors="ignore")
                    line = raw_line_string.split(',')
                    with open(log_file, "a") as f:
                        f.write(raw_line_string + '\n')
                    with lock: #avoid race condition with telemetry()
                        data = [float(x) for x in line]
            except serial.SerialException:
                print("Teensy disconnected")
                ser.close()
                ser = None

@app.route("/")
def index():
    print("New connection")
    return render_template("dashboard_test.html")

i = 0;
def get_local_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # No traffic is actually sent
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    return ip

last_time = time.time()
def telemetry():
    while True:
        global last_time;
        #print(str(1000*(time.time()-last_time)) + "request_data")
        print("Sending Data")
        last_time = time.time()
        my_data = []; #a place to copy the data so the lock is only held for a short time
        global i
        i = i + 1;
        with lock: #avoid race condition with serial_thread
            my_data = data
        '''return jsonify({
            "timestamp": time.strftime("%H:%M:%S"),
            "pressure": round(random.uniform(20, 80), 2),
            "temperature": round(random.uniform(30, 120), 2),
            "altitude": round(random.uniform(0, 2500), 2),
            "battery": round(random.uniform(10.5, 12.6), 2)
        })'''
        temp_data = [(time.time()-start_time), #0: Time
                     math.sin(i/100),  #1: Pos X
                     math.sin(i/100),  #2: Pos Y
                     math.sin(i/100),  #3: Pos Z
                     math.sin(i/100),  #4: Vel X
                     math.sin(i/100),  #5: Vel Y
                     math.sin(i/100),  #6: Vel Z
                     math.sin(i/100),  #7: Theta
                     math.sin(i/100),  #8: Phi
                     math.sin(i/100),  #9: Supply Tank Temp
                     math.sin(i/100),  #10: Rocket Tank Temp
                     math.sin(i/100),  #11: N2O Pressure
                     math.sin(i/100),  #12: N2 Pressure
                     math.sin(i/100),  #13: Load Cell Reading
                     math.sin(i/100),  #14: Ambient Temp
                     math.sin(i/100),  #15: Ambient Pressure
                     math.sin(i/100),  #16: AV1 Battery Voltage
                     math.sin(i/100),  #17: AV2 Battery Voltage
                     math.sin(i/100),  #18: GSE Battery Voltage
                     ]
        binary_data = struct.pack(str(len(temp_data))+"f", *temp_data)

        # Emit as binary payload
        socketio.emit("float_buffer", binary_data)
        socketio.sleep(0.01)

def run_flask():
    local_ip = get_local_ip()
    
    print(f"Server running:")
    print(f"  Local:   http://127.0.0.1:{port}")
    print(f"  Network: http://{local_ip}:{port}")
    socketio.start_background_task(telemetry)
    socketio.run(app, host="0.0.0.0", port=port)

# Start Flask in a separate thread
flask_thread = threading.Thread(target=run_flask, daemon=True)
flask_thread.start()

# Start the serial thread
thread = threading.Thread(target=serial_thread, daemon=True)
thread.start()
    
