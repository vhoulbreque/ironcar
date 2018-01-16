from inputs import get_gamepad
from socketIO_client import SocketIO


PORT = 8000
IP = 'http://192.168.42.1'

socketIO = SocketIO(IP, port=PORT, wait_for_connection=False)


def gamepad_ctrl():
    quit_gp = False
    while not quit_gp:
        events = get_gamepad()
        for event in events:
            if event.ev_type == 'Absolute' and event.code == 'ABS_X':
                if abs(event.state) > 9000:
                    socketIO.emit('dir', event.state / 32000.)
                else:
                    socketIO.emit('dir', 0)

            if event.ev_type == 'Absolute' and event.code == 'ABS_Z':
                if abs(event.state) > 10:
                    socketIO.emit('gas', -1)
            if event.ev_type == 'Absolute' and event.code == 'ABS_RZ':
                if abs(event.state) > 5:
                    socketIO.emit('gas', event.state / 255.)
                else:
                    socketIO.emit('gas', 0)
            if event.code == 'BTN_MODE' and event.state == 1:
                quit_gp = True



gamepad_ctrl()
