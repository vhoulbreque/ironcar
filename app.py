import io
import os
import datetime

from flask import Flask, send_from_directory, render_template, send_file
from flask_socketio import SocketIO, emit

from ironcar import Ironcar


MODELS_PATH = './models/'

app = Flask(__name__, static_url_path='/front/static', template_folder='front/templates/')
socketio = SocketIO(app)
ironcar = Ironcar()


# ------- WEB PAGES --------
@app.route('/')
def main():
	models = [os.path.join(MODELS_PATH, f) for f in os.listdir(MODELS_PATH) if f.endswith('.hdf5')]
	print('models : ', models)
	return render_template('index.html', models=models)

@app.route('/picture')
def picture():
	"""
	Generate a picture, save it, and send it to client
	"""
	path_picture = ironcar.picture()
	print('path_picture : ', path_picture)

	if path_picture:
		return send_file(path_picture,
						mimetype='image/jpg',
						as_attachment=True)


# ------- SOCKETS ----------
@socketio.on('mode_update')
def mode_update(mode):
	"""
	Change the driving mode of the car
	"""
	print('mode: ' + mode)
	ironcar.mode = mode


@socketio.on('model_update')
def model_update(model):
	"""
	Change the machine learning model used by the car
	"""
	print('model update: ' + model)
	ironcar.model = model


@socketio.on('starter')
def handle_starter():
	"""
	Start / Stop the car
	"""
	print('starter switch')
	state = ironcar.on_start()
	emit('starter_switch', {'activated':state}, namespace='/car') #switch it


@socketio.on('max_speed_update')
def update_max_speed(speed):
	"""
	Let the user defines a max speed for the car
	"""
	print('max speed update received: ' + str(speed))


@socketio.on('gas')
def handle_gas(gas):
	"""
	Send a gas order for manual mode
	"""
	print('gas order: ' + str(gas))


@socketio.on('dir')
def handle_dir(direction):
	"""
	Send a dir order for manual mode
	"""
	print('dir : ' + str(direction))


@socketio.on('streaming_starter')
def handle_streaming():
	"""
	To start / stop the streaming mode
	"""
	print('streaming switch')
	state = ironcar.switch_streaming()
	emit('stream_switch', {'activated':state}, namespace='/car') #switch it


if __name__ == '__main__':
	print('#' * 50)
	print('# IRONCAR SERVER')
	print('#' * 50)
	socketio.run(app)
