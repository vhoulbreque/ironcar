import io
import os
import datetime

from flask import Flask, send_from_directory, render_template, send_file
from flask_socketio import SocketIO, emit

import logging
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__, static_url_path='/static', static_folder='front/static/', template_folder='front/templates/')
socketio = SocketIO(app)