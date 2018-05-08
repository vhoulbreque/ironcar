import socket
import json

from flask import Flask, render_template, send_file, jsonify
from app import app, socketio
from ironcar import *

with open(CONFIG) as json_file:
    config = json.load(json_file)
    MODELS_PATH = config['models_path']


# ------- WEB PAGES --------
@app.route('/')
def main():
    """Switches to the `Main` tab on the dashboard"""

    models = []
    if os.path.isdir(MODELS_PATH):
        models = [os.path.join(MODELS_PATH, f)
                  for f in os.listdir(MODELS_PATH) if f.endswith('.hdf5')]
    print('SERVER : models : ', models)
    return render_template('index.html', models=models)


@app.route('/commands')
def commands():
    """Switches to `commands` tab on the dashboard"""

    commands = ironcar.commands
    print('SERVER : commands : ', commands)
    return render_template('commands.html', commands=commands)


@app.route('/help')
def help():
    """Display a help page"""
    return render_template('help.html')


@app.route('/picture')
def picture():
    """Takes a picture, saves it, and sends it to the client"""

    path_picture = ironcar.picture()
    print('path_picture : ', path_picture)

    if path_picture:
        r = send_file(path_picture,
                      as_attachment=True)
        r.headers["Pragma"] = "no-cache"
        r.headers["Expires"] = "0"
        r.headers['Cache-Control'] = 'public, max-age=0'
        return r
    return None


@app.route('/car_state')
def mode_update():
    """Sends the state of the car"""

    print('Sending the current state of the car')

    all_state = dict()
    all_state['mode'] = ironcar.mode
    all_state['speed_mode'] = ironcar.speed_mode
    all_state['started'] = ironcar.started
    all_state['current_model'] = ironcar.current_model
    all_state['max_speed_rate'] = ironcar.max_speed_rate
    all_state['commands'] = ironcar.commands

    return jsonify(all_state)


# ------- SOCKETS ----------
@socketio.on('mode_update')
def mode_update(mode):
    """Changes the driving mode of the car"""

    print('SERVER : mode switch from {} to {}'.format(ironcar.mode, mode))
    ironcar.switch_mode(mode)


@socketio.on('model_update')
def model_update(model):
    """Changes the machine learning model used by the car"""

    print('SERVER : model update: ' + model)
    ironcar.select_model(model)


@socketio.on('starter')
def handle_starter(value):
    """Starts/stops the car"""

    if value is None:
        value = not ironcar.started
        
    print('SERVER : starter switch from {} to {}'.format(ironcar.started, value))
    ironcar.started = value
    socketio.emit('starter_switch', {'activated': ironcar.started}, namespace='/car')


@socketio.on('speed_mode_update')
def speed_mode_update(speed_mode):
    """Lets the user selects the speed mode"""

    print('SERVER : speed mode received: ' + str(speed_mode))
    ironcar.switch_speed_mode(speed_mode)
    # TODO send a callback ?


@socketio.on('max_speed_update')
def update_max_speed(speed):
    """Lets the user defines a max speed for the car"""

    print('SERVER : max speed update received: ' + str(speed))
    ironcar.max_speed_update(speed)
    socketio.emit('max_speed_update_callback', {
                  'speed': ironcar.max_speed_rate}, namespace='/car')


@socketio.on('gas')
def handle_gas(gas):
    """Sends a gas order for manual mode"""

    print('SERVER : gas order: ' + str(gas))
    ironcar.on_gas(gas)


@socketio.on('dir')
def handle_dir(direction):
    """Sends a dir order"""

    print('SERVER : dir : ' + str(direction))
    ironcar.on_dir(direction)


@socketio.on('streaming_starter')
def handle_streaming():
    """To start/stop the streaming mode"""

    print('SERVER : streaming switch from {} to {}'.format(
        ironcar.streaming_state, not ironcar.streaming_state))
    ironcar.switch_streaming()
    socketio.emit('stream_switch', {'activated': ironcar.streaming_state},
                  namespace='/car')


@socketio.on('command_update')
def handle_config(data):
    """Handles the changes in the commands of the car and reloads the config
    parameters.
    """

    print('SERVER : command update')

    command = data['command']
    value = data['value']

    # Check for wrong inputs
    try:
        value = int(value)
    except Exception as e:
        print('`{}` cannot be cast to int'.format(value))
        return

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


if __name__ == '__main__':

    # Get the IP of the raspi
    try:
        IP = (([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")] or [[(s.connect(
            ("8.8.8.8", 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) + ["no IP found"])[0]
    except:
        IP = "NO IP FOUND"
    PORT = 5000

    print('#' * 50)
    print('# IRONCAR SERVER')
    print('# Go to the url: {}:{}'.format(IP, PORT))
    print('#' * 50)

    ironcar = Ironcar()
    socketio.run(app, host='0.0.0.0')
