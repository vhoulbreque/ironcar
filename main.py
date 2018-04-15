import socket
import json

from flask import Flask, render_template, send_file
from app import app, socketio
from ironcar import *

CONFIG = 'config.json'
with open(CONFIG) as json_file:
	config = json.load(json_file)
	MODELS_PATH = config['models_path']

# ------- WEB PAGES --------
@app.route('/')
def main():
	models = []
	if os.path.isdir(MODELS_PATH):
		models = [os.path.join(MODELS_PATH, f) for f in os.listdir(MODELS_PATH) if f.endswith('.hdf5')]
	print('SERVER : models : ', models)
	return render_template('index.html', models=models)


@app.route('/commands')
def commands():
	commands = ironcar.commands
	return render_template('commands.html', commands=commands)


@app.route('/picture')
def picture():
	"""
	Generate a picture, save it, and send it to client
	"""
	path_picture = ironcar.picture()
	print('path_picture : ', path_picture)

	if path_picture:
		r = send_file(path_picture,
						as_attachment=True)
		r.headers["Pragma"] = "no-cache"
		r.headers["Expires"] = "0"
		r.headers['Cache-Control'] = 'public, max-age=0'
		return r


# ------- SOCKETS ----------
@socketio.on('mode_update')
def mode_update(mode):
	"""
	Change the driving mode of the car
	"""
	print('SERVER : mode: ' + mode)
	ironcar.switch_mode(mode)


@socketio.on('model_update')
def model_update(model):
	"""
	Change the machine learning model used by the car
	"""
	socketio.emit('msg2user', {'type': 'info', 'msg': 'Loading model {}...'.format(model)}, namespace='/car')
	print('SERVER : model update: ' + model)
	ironcar.select_model(model)


@socketio.on('starter')
def handle_starter():
	"""
	Start / Stop the car
	"""
	print('SERVER : starter switch')
	state = ironcar.on_start()
	socketio.emit('starter_switch', {'activated': state}, namespace='/car') # switch it


@socketio.on('max_speed_update')
def update_max_speed(speed):
	"""
	Let the user defines a max speed for the car
	"""
	new_speed = ironcar.max_speed_update(speed)
	print(speed)
	print('SERVER : max speed update received: ' + str(speed))
	socketio.emit('max_speed_update_callback', {'speed': new_speed}, namespace='/car') # switch it


@socketio.on('gas')
def handle_gas(gas):
	"""
	Send a gas order for manual mode
	"""
	print('SERVER : gas order: ' + str(gas))
	ironcar.on_gas(gas)


@socketio.on('dir')
def handle_dir(direction):
	"""
	Send a dir order for manual mode
	"""
	print('SERVER : dir : ' + str(direction))
	ironcar.on_dir(direction)


@socketio.on('streaming_starter')
def handle_streaming():
	"""
	To start / stop the streaming mode
	"""
	print('SERVER : streaming switch')
	state = ironcar.switch_streaming()
	socketio.emit('stream_switch', {'activated': state}, namespace='/car') # switch it


@socketio.on('command_update')
def handle_config(data):
	"""
	To start / stop the streaming mode
	"""
	print('SERVER : command update')

	command = data['command']
	value = data['value']

	# Modify the config file
	with open(CONFIG) as json_file:
		config = json.load(json_file)

	if command not in config['commands']:
		print('The command `{}` is not available in config'.format(command))
		return

	if command == 'invert_dir':
		config['commands'][command] = int(value) * config['commands'][command]
	else:
		config['commands'][command] = int(value)

	with open(CONFIG, 'w') as fp:
		fp.write(json.dumps(config, indent=4))

	# Load the modified config file in ironcar
	ironcar.load_config()


@socketio.on('verbose')
def handle_verbose(verbose):
	"""
	Handle verbose of ironcar
	"""
	print('SERVER : verbose switch')
	ironcar.switch_verbose(verbose)


if __name__ == '__main__':

	IP = (([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")] or [[(s.connect(("8.8.8.8", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) + ["no IP found"])[0]
	PORT = 5000

	print('#' * 50)
	print('# IRONCAR SERVER')
	print('# Go to the url: {}:{}'.format(IP, PORT))
	print('#' * 50)

	ironcar = Ironcar()
	socketio.run(app, host='0.0.0.0')
