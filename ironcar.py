import os
import PIL
import json
import base64
import scipy.misc
import numpy as np

from io import BytesIO
from app import socketio
from threading import Thread
from datetime import datetime

try:
	from picamera import PiCamera
	from picamera.array import PiRGBArray
except Exception as e:
	print('Pi Camera error : ', e)

try:
	from Adafruit_PCA9685 import PCA9685
except Exception as e:
	print('Adafruit error : ', e)

try:
	from keras.models import load_model
	from tensorflow import get_default_graph
except Exception as e:
	print('ML error : ', e)


# TODO put those variables in the commands.json file ?
STREAM_PATH = './stream/'
FPS = 60
CAM_RESOLUTION = (250, 150)
COMMANDS_JSON_FILE = 'commands.json'

ct = datetime.now().strftime('%Y_%m_%d_%H_%M')
save_folder = os.path.join('datasets/', str(ct))

if not os.path.exists(save_folder):
	os.makedirs(save_folder)

try:
	# PWM setup
	pwm = PCA9685()
	pwm.set_pwm_freq(60)
except:
	pwm = None

class Ironcar():
	"""
	Class of the car. Contains all the different fields, functions needed to
	control the car.
	"""

	def __init__(self):

		if not os.path.exists(STREAM_PATH):
			os.makedirs(STREAM_PATH)

		self.mode = 'resting'
		self.started = False  # If True, car will move, if False car won't move.
		self.model = None
		self.graph = None
		self.curr_dir = 0
		self.curr_gas = 0
		self.max_speed_rate = 0.5
		self.model_loaded = False
		self.streaming_state = False
		self.n_img = 0
		self.save_number = 0
		self.current_model = None

		self.verbose = True
		self.mode_function = self.default_call

		with open(COMMANDS_JSON_FILE) as json_file:
			self.commands = json.load(json_file)

		self.camera_thread = Thread(target=self.camera_loop, args=())
		self.camera_thread.start()

	def picture(self):
		# TODO this function won't work as expected if streaming mode is off.
		# This function should take its own picture.
		pictures = sorted([f for f in os.listdir(STREAM_PATH)])
		if len(pictures):
			p = pictures[-1]
			return os.path.join(STREAM_PATH, p)
		else:
			if self.verbose:
				socketio.emit('msg2user', {'type': 'warning', 'msg': 'There is no picture to send'}, namespace='/car')
				print('There is no picture to send')
			return None

	def gas(self, value):
		"""
		Send the pwm signal on the gas channel
		"""
		if pwm is not None:
			pwm.set_pwm(self.commands['gas_pin'], 0 , value)
			if self.verbose:
				print('GAS : ', value)
		else:
			if self.verbose:
				print('PWM module not loaded')

	def dir(self, value):
		"""
		Send the pwm signal on the dir channel
		"""
		if pwm is not None:
			pwm.set_pwm(self.commands['dir_pin'], 0 , value)
			if self.verbose:
				print('DIR : ', value)
		else:
			if self.verbose:
				print('PWM module not loaded')

	def get_gas_from_dir(self, dir):
		"""
		Given the prediction of the direction by the NN, determine the gas value
		"""
		return 0.2

	def autopilot(self, img, prediction):
		"""
		Sends the pwm gas and dir values according to the prediction of the NN.

		img: unused. But has to stay because other modes need it.
		prediction: array of softmax
		"""
		index_class = prediction.index(max(prediction))
		local_dir = -1 + 2 * float(index_class)/float(len(prediction)-1)
		local_gas = self.get_gas_from_dir(local_dir) * self.max_speed_rate

		self.dir(int(local_dir * (self.commands['right'] - self.commands['left'])/2. + self.commands['straight']))
		if self.started:
			gas_value = int(local_gas * (self.commands['drive_max'] - self.commands['drive']) + self.commands['drive'])
		else:
			gas_value = self.commands['neutral']
		self.gas(gas_value)

	def dirauto(self, img, prediction):
		"""
		Set the pwm values for dir according to the prediction from the NN.
		"""
		index_class = prediction.index(max(prediction))

		local_dir = -1 + 2 * float(index_class) / float(len(prediction) - 1)
		self.dir(int(local_dir * (self.commands['right'] - self.commands['left']) / 2. + self.commands['straight']))

	def training(self, img, prediction):
		"""
		Saves the image of the picamera with the right labels of dir and gas.
		"""
		image_name = os.path.join(save_folder, 'frame_' + str(self.n_img) + '_gas_' +
								  str(self.curr_gas) + '_dir_' + str(self.curr_dir) +
								  '_' + '.jpg')
		img_arr = np.array(img[80:, :, :], copy=True)
		scipy.misc.imsave(image_name, img_arr)
		self.n_img += 1

	def default_call(self, img, prediction):
		"""
		Default function call. Does nothing.
		"""
		pass

	def switch_mode(self, new_mode):
		"""
		Switches the mode between:
			- training
			- rest
			- dirauto
			- auto
		"""

		# always switch the starter to stopped when switching mode
		self.started = False
		socketio.emit('starter_switch', {'activated': self.started}, namespace='/car') # Tell front we changed the mode.

		# Stop the gas before switching mode
		self.gas(self.commands['neutral'])

		if new_mode == "dirauto":
			self.mode = 'dirauto'
			if self.model_loaded:
				self.mode_function = self.dirauto
			else:
				if self.verbose:
					socketio.emit('msg2user', {'type': 'warning', 'msg': 'Model not loaded'}, namespace='/car')
					print("model not loaded")
		elif new_mode == "auto":
			self.mode = 'auto'
			if self.model_loaded:
				self.mode_function = self.autopilot
			else:
				if self.verbose:
					socketio.emit('msg2user', {'type': 'warning', 'msg': 'Model not loaded'}, namespace='/car')
					print("model not loaded")
		elif new_mode == "training":
			self.mode = 'training'
			self.mode_function = self.training
		else:
			self.mode = 'resting'
			self.mode_function = self.default_call

		# Make sure we stop even if the previous mode sent a last command before switching.
		self.gas(self.commands['neutral'])
		if self.verbose:
			print('switched to mode : ', new_mode)

	def on_start(self):
		"""
		Switch started mode between True and False.
		"""
		self.started = not self.started
		if self.verbose:
			print('starter set to {}'.format(self.started))
		return self.started

	def on_dir(self, data):
		"""
		Triggered when a value from the keyboard/gamepad is received for dir.
		data: intensity of the key pressed.
		"""

		if self.mode not in ['training']:  # Ignore dir commands if not in training mode
			if self.verbose:
				print('Ignoring dir command')
			return

		self.curr_dir = self.commands['invert_dir'] * float(data)
		if self.curr_dir == 0:
			new_value = self.commands['straight']
		else:
			new_value = int(self.curr_dir * (self.commands['right'] - self.commands['left'])/2. + self.commands['straight'])
		self.dir(new_value)

	def on_gas(self, data):
		"""
		Triggered when a value from the keyboard/gamepad is received for gas.
		data: intensity of the key pressed.
		"""

		if self.mode not in ['training', 'dirauto']:  # Ignore gas commands if not in training/dirauto mode
			if self.verbose:
				print('Ignoring gas command')
			return

		self.curr_gas = float(data) * self.max_speed_rate

		if self.curr_gas < 0:
			new_value = self.commands['stop']
		elif self.curr_gas == 0:
			new_value = self.commands['neutral']
		else:
			new_value = int(self.curr_gas * (self.commands['drive_max']-self.commands['drive']) + self.commands['drive'])
		self.gas(new_value)

	def max_speed_update(self, new_max_speed):
		"""
		Changes the max_speed of the car.
		"""
		self.max_speed_rate = new_max_speed
		if self.verbose:
			print('The new max_speed is : ', self.max_speed_rate)
		return self.max_speed_rate

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
				if self.verbose:
					print('pred : ', pred)
			prediction = list(pred[0])
		except Exception as e:
			if self.verbose:
				pass#print('Prediction error : '+e)
			prediction = [0, 0, 1, 0, 0]

		return prediction

	def camera_loop(self):
		"""
		Makes the camera take pictures and save them.
		This loop will be executed in a separate thread.
		"""

		try:
			cam = PiCamera(framerate=FPS)
		except Exception as e:
			# TODO improve
			if self.verbose:
				print('Cant load camera')
			return

		cam.resolution = CAM_RESOLUTION
		cam_output = PiRGBArray(cam, size=CAM_RESOLUTION)
		stream = cam.capture_continuous(cam_output, format="rgb", use_video_port=True)

		for f in stream:
			img_arr = f.array
			image_name = os.path.join(STREAM_PATH, 'capture.jpg')
			scipy.misc.imsave(image_name, img_arr)

			prediction = self.predict_from_img(img_arr)
			self.mode_function(img_arr, prediction)

			if self.streaming_state:
				index_class = prediction.index(max(prediction))
				img_arr = PIL.Image.fromarray(img_arr)

				buffered = BytesIO()
				img_arr.save(buffered, format="JPEG")
				img_str = base64.b64encode(buffered.getvalue())
				socketio.emit('picture_stream', {'image': True, 'buffer': img_str.decode('ascii'), 'index': index_class, 'pred': [float(x) for x in prediction]}, namespace='/car')

			cam_output.truncate(0)

	def switch_streaming(self):
		"""
		Switches the streaming state.
		"""
		self.streaming_state = not self.streaming_state
		if self.verbose:
			print('Streaming state set to {}'.format(self.streaming_state))
		return self.streaming_state

	def select_model(self, model_name):
		"""
		Changes the model of autopilot selected and loads it.
		"""

		if model_name == self.current_model:
			socketio.emit('msg2user', {'type': 'info', 'msg': 'Model {} already loaded.'.format(self.current_model)}, namespace='/car')
			return 0

		try:
			print(model_name)
			self.model = load_model(model_name)
			self.graph = get_default_graph()
			self.current_model = model_name

			self.model_loaded = True
			self.switch_mode(self.mode)

			if self.verbose:
				socketio.emit('msg2user', {'type': 'success', 'msg': 'The model {} has been successfully loaded'.format(self.current_model)}, namespace='/car')
				print('The model {} has been successfully loaded'.format(self.current_model))
		except OSError as e:
			if self.verbose:
				print('An Exception occured : ', e)

	def switch_verbose(self, new_verbose):
		if self.verbose:
			print('Switch verbose from {} to {}'.format(self.verbose, new_verbose))
		self.verbose = new_verbose
