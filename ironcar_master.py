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
models_path = './autopilots/'

fps = 60

cam_resolution = (250, 150)

commands_json_file = "commands.json"

# ***********************************************************************************

# --------------------------- SETUP ------------------------

ct = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
save_folder = os.path.join('datasets/', str(ct))

if not os.path.exists(save_folder):
    os.makedirs(save_folder)

with open(commands_json_file) as json_file:
    commands = json.load(json_file)

# PWM setup
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)

state, mode, running = "stop", "training",  True
n_img = 0
curr_dir, curr_gas = 0, 0 
model_loaded = False


# ---------------- Different modes functions ----------------


def get_gas_from_dir(dir):
    return 0.2


def default_call(img):
    pass


def autopilot(img):
    global model, graph

    img = np.array([img[80:, :, :]])
    with graph.as_default():
        pred = model.predict(img)
        print('pred : ', pred)
        prediction = list(pred[0])
    index_class = prediction.index(max(prediction))

    local_dir = -1 + 2 * float(index_class)/float(len(prediction)-1)
    local_gas = get_gas_from_dir(curr_dir)

    pwm.set_pwm(commands['direction'], 0, int(local_dir * (commands['right'] - commands['left'])/2. + commands['straight']))
    if state == "start":
        pwm.set_pwm(commands['gas'], 0, int(local_gas * (commands['drive_max'] - commands['drive']) + commands['drive']))
    else:
        pwm.set_pwm(commands['gas'], 0, commands['neutral'])


def dirauto(img):
    global model, graph

    img = np.array([img[80:, :, :]])
    with graph.as_default():
        pred = model.predict(img)
        print('pred : ', pred)
        prediction = list(pred[0])
    index_class = prediction.index(max(prediction))

    local_dir = -1 + 2 * float(index_class) / float(len(prediction) - 1)
    pwm.set_pwm(commands['direction'], 0,
                int(local_dir * (commands['right'] - commands['left']) / 2. + commands['straight']))


def training(img):
    global n_img, curr_dir, curr_gas
    image_name = os.path.join(save_folder, 'frame_' + str(n_img) + '_gas_' +
                              str(curr_gas) + '_dir_' + str(curr_dir) +
                              '_' + '.jpg')
    img_arr = np.array(img[80:, :, :], copy=True)
    scipy.misc.imsave(image_name, img_arr)
    n_img += 1


# ------------------- Main camera loop  ---------------------
# This function is launched on a separate thread that is supposed to run permanently
# to get camera pics
def camera_loop():
    global state, mode_function, running

    cam = picamera.PiCamera(framerate=fps)
    cam.resolution = cam_resolution
    cam_output = picamera.array.PiRGBArray(cam, size=cam_resolution)
    stream = cam.capture_continuous(cam_output, format="rgb", use_video_port=True)
    
    for f in stream:
        img_arr = f.array
        if not running:
            break
        mode_function(img_arr)

        cam_output.truncate(0)


# ------------------ SocketIO callbacks-----------------------
def on_model_selected(model_name):
    global models_path, model_loaded, model, graph
    model_loaded = True
    new_model_path = models_path + model_name + ".hdf5"
    print('Loading model at path : ', new_model_path)
    model = load_model(new_model_path)
    graph = tf.get_default_graph()
    print('Finished loading model')


def on_switch_mode(data):
    global mode, mode_function, model_loaded, model, graph
    mode = data
    if data == "dirauto":
        socketIO.off('dir')
        if model_loaded:
            mode_function = dirauto
        else:
            print("model not loaded")
            socketIO.emit('msg2user', 'Please load a model first')
    elif data == "auto":
        socketIO.off('gas')
        socketIO.off('dir')
        if model_loaded:
            mode_function = autopilot
        else:
            print("model not loaded")
            socketIO.emit('msg2user', 'Please load a model first')
    elif data == "training":

        socketIO.on('gas', on_gas)
        socketIO.on('dir', on_dir)
        mode_function = training
    else: 
        mode_function = default_call
    print('switched to mode : ', data)


def on_start(data):
    global state
    state = data
    print('starter set to  ' + data)


def on_dir(data):
    global curr_dir
    curr_dir = float(data)
    if curr_dir == 0:
        #print(commands['straight'])
        pwm.set_pwm(commands['direction'], 0 , commands['straight'])
    else:
        #print(int(curr_dir * (commands['right'] - commands['left'])/2. + commands['straight']))
        pwm.set_pwm(commands['direction'], 0 , int(curr_dir * (commands['right'] - commands['left'])/2. + commands['straight']))


def on_gas(data):
    global curr_gas
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
socketIO.on('model_update', on_model_selected)
socketIO.on('starter', on_start)
socketIO.on('gas', on_gas)
socketIO.on('dir', on_dir)

try:
    socketIO.wait()
except KeyboardInterrupt:
    running = False
    camera_thread.join()
