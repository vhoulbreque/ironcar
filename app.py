from flask import Flask, send_from_directory, render_template
from flask_socketio import SocketIO, emit

app = Flask(__name__)
socketio = SocketIO(app)

# ------- WEB PAGES --------
@app.route('/')
def hello_world():
	return render_template('index.html')

# ------- WEB PAGES utils --
@app.route('/<path:path>')
def send_public(path):
	return send_from_directory('public/', path)

# ------- SOCKETS ----------
@socketio.on('mode_update')
def mode_update(mode):
	"""
	Change the driving mode of the car
	"""
	print('mode: ' + mode)

@socketio.on('model_update')
def model_update(model):
	"""
	Change the machine learning model used by the car
	"""
	print('model update: ' + model)

@socketio.on('starter')
def handle_starter():
	"""
	Start / Stop the car
	"""
	print('starter switch')

	emit('starter_switch', {'activated':False}) #switch it


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

	emit('stream_switch', {'activated':False}) #switch it

@socketio.on('take_picture')
def take_picture():
	"""
	To take a picture
	"""
	print('TAKE A PICTURE')

	emit('picture', {'base64':'dede'})
	

if __name__ == '__main__':
	print('#' * 50)
	print('# IRONCAR SERVER')
	print('#' * 50)
	socketio.run(app)



