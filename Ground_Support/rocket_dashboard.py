import os
import time
import random
import threading
import serial
import serial.tools.list_ports  # optional, for listing ports
from flask import Flask, jsonify, render_template, Response, redirect
from flask_socketio import SocketIO, emit
import socket
import struct
import serial.tools.list_ports
import math

send_command = False
command_to_be_sent = "0"

stop_event = threading.Event()

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
    log_file =  str(int(max(filenumbers)) + 1)+ ".txt"
else:
    log_file = "0.txt"

TARGET_KEYWORD = "1A86"   # or use VID like "16C0"
#TARGET_KEYWORD = "16C0"   # or use VID like "16C0"
BAUD = 115200
def serial_thread():
    global data
    global send_command
    global command_to_be_sent
    ser = None
    while True:
        # Check if serial is open
        if ser is None or not ser.is_open:
            port = None
            for p in serial.tools.list_ports.comports():
                print("Available Serial Devices")
                print(p.description);
                print(p.hwid);
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
                    raw_line = ser.readline()          # bytes, NOT str
                    raw_line = raw_line[0:len(raw_line)-2:1]
                    line = raw_line.split(b',')        # split using bytes
                    data = [
                        struct.unpack('<f', s)[0]
                        for s in line
                        if len(s) == 4
                    ]
                    with open(path + "/" + log_file, "ab") as f:
                        for i in data:
                            f.write(f"{i},".encode('ascii'))
                        f.write(b'\n')      # log raw bytes safely

                    #print([len(s) for s in line])
                    #print(line)
                    #print(data)
                    #print("recv Data")
            except (serial.SerialException, OSError):
                print("Teensy disconnected")
                ser.close()
                ser = None
            if send_command:
                    ser.write(command_to_be_sent.encode('ascii'))
                    send_command = False


@app.route("/")
def index():
    print("New connection")
    return render_template("dashboard_test.html")

@app.route("/chart.js")
def redirect_chart():
    local_ip = get_local_ip()
    return redirect(f"http://{local_ip}:3000/chart.js")

@app.route("/socket.io")
def redirect_socket():
    local_ip = get_local_ip()
    return redirect(f"http://{local_ip}:3000/socket.io/socket.io.js")

@app.route("/1", methods=['POST'])
def cmd1():
    cmd("1")
    return ""
@app.route("/2", methods=['POST'])
def cmd2():
    cmd("2")
    return ""
@app.route("/3", methods=['POST'])
def cmd3():
    cmd("3")
    return ""
@app.route("/4", methods=['POST'])
def cmd4():
    cmd("4")
    return ""
@app.route("/5", methods=['POST'])
def cmd5():
    cmd("5")
    return ""
@app.route("/6", methods=['POST'])
def cmd6():
    cmd("6")
    return ""
@app.route("/7", methods=['POST'])
def cmd7():
    cmd("7")
    return ""   
@app.route("/8", methods=['POST'])
def cmd8():
    cmd("8")
    return ""
@app.route("/9", methods=['POST'])
def cmd9():
    cmd("9")
    return ""
@app.route("/A", methods=['POST'])
def cmdA():
    cmd("A")
    return ""
@app.route("/B", methods=['POST'])
def cmdB():
    cmd("B")
    return ""
@app.route("/C", methods=['POST'])
def cmdC():
    cmd("C")
    return ""
@app.route("/D", methods=['POST'])
def cmdD():
    cmd("D")
    return ""
@app.route("/E", methods=['POST'])
def cmdE():
    cmd("E")
    return ""
@app.route("/F", methods=['POST'])
def cmdF():
    cmd("F")
    return ""
@app.route("/G", methods=['POST'])
def cmdG():
    cmd("G")
    return ""
@app.route("/H", methods=['POST'])
def cmdH():
    cmd("H")
    return ""
@app.route("/I", methods=['POST'])
def cmdI():
    cmd("I")
    return ""
@app.route("/J", methods=['POST'])
def cmdJ():
    cmd("J")
    return ""
@app.route("/K", methods=['POST'])
def cmdK():
    cmd("K")
    return ""
@app.route("/L", methods=['POST'])
def cmdL():
    cmd("L")
    return ""
@app.route("/M", methods=['POST'])
def cmdM():
    cmd("M")
    return ""

def cmd(num_string):
    global command_to_be_sent
    global send_command
    send_command = True
    command_to_be_sent = num_string


i = 0
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
        #print("Sending Data ")
        last_time = time.time()
        my_data = []; #a place to copy the data so the lock is only held for a short time
        global i
        i = i + 1;
        #with lock: #avoid race condition with serial_thread
        my_data = data
        '''return jsonify({
            "timestamp": time.strftime("%H:%M:%S"),
            "pressure": round(random.uniform(20, 80), 2),
            "temperature": round(random.uniform(30, 120), 2),
            "altitude": round(random.uniform(0, 2500), 2),
            "battery": round(random.uniform(10.5, 12.6), 2)
        })'''
        if len(my_data) == 34:
            temp_data = [(time.time()-start_time), #0: Time
                         data[0],  #1: Pos X
                         data[1],  #2: Pos Y
                         data[2],  #3: Pos Z
                         data[3],  #4: Vel X
                         data[4],  #5: Vel Y
                         data[5],  #6: Vel Z
                         data[6],  #7: Orientation Theta
                         data[7],  #8: Supply Tank Temp
                         data[8],  #9: Rocket Tank Temp
                         data[9],  #10: N2O Pressure
                         data[10],  #11: N2 Pressure
                         data[11],  #12: Supply Tank Load Cell Reading
                         data[12],  #13: Rocket Tank Load Cell Reading
                         data[13],  #14: Command count
                         data[14],  #15: Ambient Temp
                         data[15],  #16: Ambient Pressure
                         data[16],  #17: AV1 Ox-Tank Pressure
                         data[17],  #18: AV2 CO2 Pressure
                         data[18],  #19: AV1 Battery Voltage
                         data[19],  #20: AV2 Battery Voltage
                         data[20],  #21: N2O Valve State
                         data[21],  #22: N2 Valve State
                         data[22],  #23: Poppet Valve State
                         data[23],  #24: Quick Disconnect State
                         data[24],  #25: Clamshell State
                         data[25],  #26: AC Unit State
                         data[26],  #27: GSE RSSI
                         data[27],  #28: AV1 RSSI
                         data[28],  #29: AV2 RSSI
                         data[29],  #30: GSE ACK
                         data[30],  #31: AV1 ACK
                         data[31],  #32: AV2 ACK
                         data[32],  #33: Passive Vent Valve State
                         data[33],  #34: Dump Vent Valve State
                         ]
            #print(data)
            binary_data = struct.pack(str(len(temp_data))+"f", *temp_data)
            #print("send data")
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
flask_thread = threading.Thread(target=run_flask, daemon=False)
flask_thread.start()

# Start the serial thread
thread = threading.Thread(target=serial_thread, daemon=False)
thread.start()
    
