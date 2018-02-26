from flask import Flask, send_from_directory, render_template, send_file
from app import app, socketio
from ironcar import *


MODELS_PATH = './models/'

# ------- WEB PAGES --------
@app.route('/')
def main():
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


@socketio.on('verbose')
def handle_verbose(verbose):
	"""
	Handle verbose of ironcar
	"""
	print('SERVER : verbose switch')
	ironcar.switch_verbose(verbose)

if __name__ == '__main__':
	print('#' * 50)
	print('# IRONCAR SERVER')
	print('#' * 50)
	ironcar = Ironcar()
	socketio.run(app, host='0.0.0.0')
	
