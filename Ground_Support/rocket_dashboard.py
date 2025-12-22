import os
import time
import random
from flask import Flask, jsonify, render_template

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__, template_folder=os.path.join(BASE_DIR, "templates"))

@app.route("/")
def index():
    return render_template("dashboard.html")

@app.route("/telemetry")
def telemetry():
    return jsonify({
        "timestamp": time.strftime("%H:%M:%S"),
        "pressure": round(random.uniform(20, 80), 2),
        "temperature": round(random.uniform(30, 120), 2),
        "altitude": round(random.uniform(0, 2500), 2),
        "battery": round(random.uniform(10.5, 12.6), 2)
    })
