from flask import Flask
from flask_socketio import SocketIO

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__, static_url_path='/static', static_folder='front/static/', template_folder='front/templates/')
socketio = SocketIO(app)
