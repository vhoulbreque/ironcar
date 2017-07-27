from socketIO_client import SocketIO
from threading import Thread

import sys, os, time
import scipy.misc
import datetime

import picamera
import picamera.array
from PIL import Image

from Adafruit_BNO055 import BNO055
import Adafruit_PCA9685
#from utils import ArgumentError, initialize_imu

from keras.models import load_model
import tensorflow as tf
import numpy as np
import json

# *********************************** Parameters ************************************
model_path = 'autopilots/autopilot_500k.hdf5'

fps = 60

cam_resolution = (250, 150)

commands_json_file = "commands.json"
#model_input_size = ()
# ***********************************************************************************

# --------------------------- SETUP ------------------------

with open(commands_json_file) as json_file:
    commands = json.load(json_file)

# PWM setup
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

global state, mode, running
state, mode, running = "stop","training",  True
model_loaded = False

# ---------------- Different modes functions ----------------
def default_call(img):
    pass


def autopilot(img):
    global model, graph
    cache = time.time()
    img = np.array([img[80:, :, :]])
    with graph.as_default():
        pred = model.predict(img)
        print('pred : ', pred)
        prediction = list(pred[0])
    index_class = prediction.index(max(prediction))
    curr_dir = -1 + 2 * float(index_class)/float(len(prediction)-1)
    #curr_gas = get_gas_from_dir(curr_dir)
    #print("curr_gas ", curr_gas)
    pwm.set_pwm(commands['direction'], 0 , int(curr_dir * (commands['right'] - commands['left'])/2. + commands['straight']))


def training(img):
    global curr_dir, curr_gas
    
    img_name = "frame" + "_gas_" + str(curr_gas) + "_dir_" + str(curr_dir)
    pass


# ------------------- Main camera loop  ---------------------
# This function is launched on a separate thread that is supposed to run permanently
# to get camera pics
def camera_loop():
    global state, mode_function, running
    i = 0

    cam = picamera.PiCamera(framerate=fps)
    cam.resolution = cam_resolution
    cam_output = picamera.array.PiRGBArray(cam, size=cam_resolution)
    stream = cam.capture_continuous(cam_output, format="rgb", use_video_port=True)
    
    for f in stream:
        img_arr = f.array
        if not running: break
        if state == "start":
            mode_function(img_arr)

        #print("state, mode are: ", state)
        #time.sleep(0.3)
        cam_output.truncate(0)


# ------------------ SocketIO callbacks-----------------------
def on_switch_mode(data):
    global mode, mode_function, model_loaded, model, graph
    mode = data
    if data == "dir_auto":
        if not model_loaded:
            model_loaded = True
            print('Loading model at path : ', model_path)
            model = load_model(model_path)
            graph = tf.get_default_graph()
            print('Finished loading model')

        mode_function = autopilot
    elif data == "training":
        mode_function = training
    else: 
        mode_function = default_call
    print('switched to mode : ', data)


def on_start(data):
    global state
    state = data
    print('starter set to  ' + data)


def on_dir(data):
    global mode, curr_dir
    if mode == "training":
#        msg_in = data.split("_")
        curr_dir = float(data)   
#        curr_gas = float(msg_in[1])
#        curr_dir = float(msg_in[3])
        if curr_dir == 0:
            #print(commands['straight'])
            pwm.set_pwm(commands['direction'], 0 , commands['straight'])
        else:
            #print(int(curr_dir * (commands['right'] - commands['left'])/2. + commands['straight']))
            pwm.set_pwm(commands['direction'], 0 , int(curr_dir * (commands['right'] - commands['left'])/2. + commands['straight']))


def on_gas(data):
    global mode, curr_gas
    if mode == "training" or mode == "dir_auto":

        curr_gas = float(data)
        if curr_gas < 0:
            pwm.set_pwm(commands['gas'], 0 , commands['stop'])
        elif curr_gas == 0:
            pwm.set_pwm(commands['gas'], 0 , commands['neutral'])
        else:
            #print(curr_gas * (commands['drive_max'] - commands['drive']) + commands['drive'])
            pwm.set_pwm(commands['gas'], 0 , int(curr_gas * (commands['drive_max']-commands['drive']) + commands['drive']))
    

# --------------- Starting server and threads ----------------
mode_function = default_call
socketIO = SocketIO('http://localhost', port=8000, wait_for_connection=False)

camera_thread = Thread(target=camera_loop, args=())
camera_thread.start()
socketIO.on('mode_update', on_switch_mode)
socketIO.on('starter', on_start)
socketIO.on('gas', on_gas)
socketIO.on('dir', on_dir)

try:
    socketIO.wait()
except KeyboardInterrupt:
    running = False
    camera_thread.join()
