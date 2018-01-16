from socketIO_client import SocketIO
from threading import Thread

import sys, os, time
import scipy.misc
import datetime

import picamera
import picamera.array

from Adafruit_BNO055 import BNO055
import Adafruit_PCA9685

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

state, mode, running = "stop", "resting",  True
n_img = 0
curr_dir, curr_gas = 0, 0
current_model = None
max_speed_rate = 0.5
model_loaded = False


# ---------------- Different modes functions ----------------


def get_gas_from_dir(dir):
    return 0.2


def default_call(img):
    pass


def autopilot(img):
    global model, graph, state, max_speed_rate

    img = np.array([img[80:, :, :]])
    with graph.as_default():
        pred = model.predict(img)
        print('pred : ', pred)
        prediction = list(pred[0])
    index_class = prediction.index(max(prediction))

    local_dir = -1 + 2 * float(index_class)/float(len(prediction)-1)
    local_gas = get_gas_from_dir(curr_dir) * max_speed_rate

    pwm.set_pwm(commands['direction'], 0, int(local_dir * (commands['right'] - commands['left'])/2. + commands['straight']))
    if state == "started":
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
# This will try to load a model when receiving a callback from the node server
def on_model_selected(model_name):
    global current_model, models_path, model_loaded, model, graph, mode
    if model_name == current_model or model_name == -1: return 0
    new_model_path = models_path + model_name
    socketIO.emit('msg2user', 'Loading model at path : ' + str(new_model_path))
    try:
        model = load_model(new_model_path)
        graph = tf.get_default_graph()
        current_model = model_name
        socketIO.emit('msg2user', ' Model Loaded!')
        model_loaded = True
        on_switch_mode(mode)
    except OSError:
        socketIO.emit('msg2user', ' Failed loading model. Please select another one.')


def on_switch_mode(data):
    global mode, state, mode_function, model_loaded, model, graph
    # always switch the starter to stopped when switching mode
    if state == "started":
        state = "stopped"
        socketIO.emit('starter')
    # Stop the gas before switching mode
    pwm.set_pwm(commands['gas'], 0 , commands['neutral'])
    mode = data
    if data == "dirauto":
        socketIO.off('dir')
        if model_loaded:
            mode_function = dirauto
            socketIO.emit('msg2user', ' Direction auto mode. Please control the gas using a keyboard or a gamepad.')
        else:
            print("model not loaded")
            socketIO.emit('msg2user', ' Please load a model first')
    elif data == "auto":
        socketIO.off('gas')
        socketIO.off('dir')
        if model_loaded:
            mode_function = autopilot
            socketIO.emit('msg2user', ' Autopilot mode. Use the start/stop button to free the gas command.')
        else:
            print("model not loaded")
            socketIO.emit('msg2user', 'Please load a model first')
    elif data == "training":
        socketIO.on('gas', on_gas)
        socketIO.on('dir', on_dir)
        mode_function = training
        socketIO.emit('msg2user', ' Training mode. Please use a keyboard or a gamepad for control.')
    else: 
        mode_function = default_call
        socketIO.emit('msg2user', ' Resting')
    print('switched to mode : ', data)
    # Make sure we stop even if the previous mode sent a last command before switching.
    pwm.set_pwm(commands['gas'], 0 , commands['neutral'])


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
    global curr_gas, max_speed_rate
    curr_gas = float(data) * max_speed_rate
    if curr_gas < 0:
        pwm.set_pwm(commands['gas'], 0, commands['stop'])
    elif curr_gas == 0:
        pwm.set_pwm(commands['gas'], 0, commands['neutral'])
    else:
        #print(curr_gas * (commands['drive_max'] - commands['drive']) + commands['drive'])
        pwm.set_pwm(commands['gas'], 0, int(curr_gas * (commands['drive_max']-commands['drive']) + commands['drive']))
    

def on_max_speed_update(new_max_speed):
    global max_speed_rate
    max_speed_rate = new_max_speed

# --------------- Starting server and threads ----------------
mode_function = default_call
socketIO = SocketIO('http://localhost', port=8000, wait_for_connection=False)
socketIO.emit('msg2user', 'Starting Camera thread')
camera_thread = Thread(target=camera_loop, args=())
camera_thread.start()
socketIO.emit('msg2user', 'Camera thread started! Please select a mode.')
socketIO.on('mode_update', on_switch_mode)
socketIO.on('model_update', on_model_selected)
socketIO.on('starterUpdate', on_start)
socketIO.on('maxSpeedUpdate', on_max_speed_update)
socketIO.on('gas', on_gas)
socketIO.on('dir', on_dir)

try:
    socketIO.wait()
except KeyboardInterrupt:
    running = False
    camera_thread.join()
