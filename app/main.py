from flask import Flask, request, render_template, redirect, jsonify, Response
# from camera import VideoCamera

import json
import serial
import time
import RPi.GPIO as GPIO

# ser = serial.Serial('/dev/ttyACM0', 9600)
# ser = serial.Serial('/dev/ttyAMA0', 9600)
ser = serial.Serial('/dev/ttyUSB0', 9600)

GPIO.setmode(GPIO.BOARD)
# GPIO.setup(pin, GPIO.OUT)

app = Flask(__name__)

@app.route('/')
def index():
  return render_template('index.html')

# Movements
# -------------------------------------------------
@app.route('/move/<state>')
def move(state):
  if state.lower() == 'forward':
    ser.write("MoveForward\n")
    return jsonify({ 'move': 'forward' })
  elif state.lower() == 'backward':
    ser.write("MoveBackward\n")
    return jsonify({ 'move': 'backward' })
  else:
    ser.write("MoveStop\n")
    return jsonify({ 'move': 'stop' })

@app.route('/turn/<state>')
def turn(state):
  if state.lower() == 'left':
    ser.write("TurnLeft\n")
    return jsonify({ 'turn': 'left' })
  elif state.lower() == 'right':
    ser.write("TurnRight\n")
    return jsonify({ 'turn': 'right' })
  else:
    ser.write("TurnCenter\n")
    return jsonify({ 'turn': 'center' })
# -------------------------------------------------

if __name__ == "__main__":
  app.run(host='192.168.1.79', threaded=True, debug=True)
  # app.run(host='0.0.0.0', threaded=True)
