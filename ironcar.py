from threading import Thread

import sys
import os
import time
import datetime
import scipy.misc
import json
import numpy as np

import picamera
import picamera.array

try:
    from Adafruit_BNO055 import BNO055
    import Adafruit_PCA9685

    from keras.models import load_model
    import tensorflow as tf
except Exception as e:
    print('An exception occured : ', e)


MODELS_PATH = './autopilots/'
FPS = 60
CAM_RESOLUTION = (250, 150)
COMMANDS_JSON_FILE = 'commands.json'

ct = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
save_folder = os.path.join('datasets/', str(ct))

if not os.path.exists(save_folder):
    os.makedirs(save_folder)

# PWM setup
pwm = Adafruit_PCA9685.PCA9685()
pwm.set_pwm_freq(60)


class Ironcar():


    def __init__(self):

        self.mode = 'resting'
        self.state = 'stop'
        self.running = True
        self.model = None
        self.graph = None
        self.curr_dir = 0
        self.curr_gas = 0
        self.max_speed_rate = 0.5
        self.model_loaded = False
        self.streaming_state = True
        self.n_img = 0
        self.save_number = 0

        self.mode_function = default_call

        with open(COMMANDS_JSON_FILE) as json_file:
            self.commands = json.load(json_file)

        self.camera_thread = Thread(target=self.camera_loop, args=())
        self.camera_thread.start()

    def gas(self, value):
        pwm.set_pwm(self.commands['gas_pin'], 0 , value)

    def dir(self, value):
        pwm.set_pwm(self.commands['dir_pin'], 0 , value)

    def get_gas_from_dir(self, dir):
        return 0.2

    def autopilot(self, img, prediction):
        index_class = prediction.index(max(prediction))
        local_dir = -1 + 2 * float(index_class)/float(len(prediction)-1)
        local_gas = get_gas_from_dir(curr_dir) * max_speed_rate

        self.dir(int(local_dir * (self.commands['right'] - self.commands['left'])/2. + self.commands['straight']))
        gas_value = 0
        if self.state == "started":
            gas_value = int(local_gas * (self.commands['drive_max'] - self.commands['drive']) + self.commands['drive'])
        else:
            gas_value = self.commands['neutral']
        self.gas(gas_value)

    def dirauto(self, img, prediction):
        index_class = prediction.index(max(prediction))

        local_dir = -1 + 2 * float(index_class) / float(len(prediction) - 1)
        self.dir(int(local_dir * (self.commands['right'] - self.commands['left']) / 2. + self.commands['straight']))

    def training(self, img, prediction):
        global n_img
        image_name = os.path.join(self.save_folder, 'frame_' + str(n_img) + '_gas_' +
                                  str(self.curr_gas) + '_dir_' + str(self.curr_dir) +
                                  '_' + '.jpg')
        img_arr = np.array(img[80:, :, :], copy=True)
        scipy.misc.imsave(image_name, img_arr)
        n_img += 1

    def default_call(self, img, prediction):
        pass

    def on_switch_mode(self, new_mode):

        # always switch the starter to stopped when switching mode
        if self.state == "started":
            self.state = "stopped"

        # Stop the gas before switching mode
        self.gas(self.commands['neutral'])

        if new_mode == "dirauto":
            if self.model_loaded:
                self.mode_function = self.dirauto
            else:
                print("model not loaded")
        elif new_mode == "auto":
            if self.model_loaded:
                self.mode_function = self.autopilot
            else:
                print("model not loaded")
        elif new_mode == "training":
            self.mode_function = self.training
        else:
            self.mode_function = self.default_call

        # Make sure we stop even if the previous mode sent a last command before switching.
        self.gas(commands['neutral'])
        print('switched to mode : ', new_mode)

    def on_start(data):
        self.state = data
        print('starter set to {}'.format(data))

    def on_dir(data):
        self.curr_dir = self.commands['invert_dir'] * float(data)
        new_value = 0
        if self.curr_dir == 0:
            new_value = self.commands['straight']
        else:
            new_value = int(self.curr_dir * (self.commands['right'] - self.commands['left'])/2. + self.commands['straight'])
        self.dir(new_value)

    def on_gas(data):
        self.curr_gas = float(data) * self.max_speed_rate

        new_value = 0
        if self.curr_gas < 0:
            new_value = self.commands['stop']
        elif self.curr_gas == 0:
            new_value = self.commands['neutral']
        else:
            new_value = int(self.curr_gas * (self.commands['drive_max']-self.commands['drive']) + self.commands['drive'])
        self.gas(new_value)

    def on_max_speed_update(new_max_speed):
        self.max_speed_rate = new_max_speed

    def predict_from_img(self, img):
        """
        Given the 250x150 image from the Pi Camera
        Returns the direction predicted by the model
        array[5] : prediction
        """

        try:
            img = np.array([img[80:, :, :]])

            with self.graph.as_default():
                pred = self.model.predict(img)
                print('pred : ', pred)
            prediction = list(pred[0])
        except:
            prediction = [0, 0, 1, 0, 0]

        return prediction

    def camera_loop(self):

        cam = picamera.PiCamera(framerate=FPS)
        cam.resolution = CAM_RESOLUTION
        cam_output = picamera.array.PiRGBArray(cam, size=CAM_RESOLUTION)
        stream = cam.capture_continuous(cam_output, format="rgb", use_video_port=True)

        for f in stream:
            img_arr = f.array
            if not self.running:
                break

            prediction = self.predict_from_img(img_arr)
            self.mode_function(img_arr, prediction)

            if self.streaming_state:
                str_n = '0'*(5-len(str(self.save_number))) + str(self.save_number)
                index_class = prediction.index(max(prediction))
                image_name = './stream/image_stream_{}_{}.jpg'.format(str_n, index_class)
                #print('Saving image at path : ', image_name)
                self.save_number += 1
                scipy.misc.imsave(image_name, img_arr)
                socketIO.emit(image_name)

            cam_output.truncate(0)

    def on_model_selected(self, model_name):
        global MODELS_PATH

        if model_name == self.current_model or model_name == -1: return 0
        new_model_path = os.path.join(models_path, model_name)

        try:
            self.model = load_model(new_model_path)
            self.graph = tf.get_default_graph()
            self.current_model = model_name

            self.model_loaded = True
            self.on_switch_mode(self.mode)
        except OSError as e:
            print('An Exception occured : ', e)
