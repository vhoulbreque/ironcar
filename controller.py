from inputs import get_gamepad
from socketIO_client import SocketIO


socketIO = SocketIO('http://192.168.42.1', port=8000, wait_for_connection=False)


def gamepad_ctrl():
    quit_gp = False
    gas, dir = 0, 0

    while not quit_gp:
        events = get_gamepad()
        for event in events:
            if event.ev_type == 'Absolute' and event.code == 'ABS_X':
                if abs(event.state) > 9000:

                    dir = event.state / 32000.
                else:
                    dir =0
            if event.ev_type == 'Absolute' and event.code == 'ABS_Z':
                if abs(event.state) > 10:
                    gas = -1
            if event.ev_type == 'Absolute' and event.code == 'ABS_RZ':
                if abs(event.state) > 5:
                    gas = event.state / 255.
                else:
                    gas = 0
            if event.code == 'BTN_MODE' and event.state == 1:
                quit_gp = True

            msg = "gas_" + str(gas) + "_dir_" + str(dir)
            socketIO.emit('client_commands', msg)


gamepad_ctrl()
