"""

The remote controller obviously controls the car from distance.
3 modes are available:
    - keyboard controls with the directional keys (UP, DOWN, LEFT and RIGHT)
    - gamepad controller (only tested with XBox 360 controller)
    - autopilot (which is not a remote control...)

Launched on the laptop.

"""

import os
import time
import sys
import rospy

from Tkinter import *
from inputs import get_gamepad
from std_msgs.msg import String, Float32, Bool

import numpy as np
import tensorflow as tf
import roslib
import scipy.misc

from PIL import Image
from sensor_msgs.msg import CompressedImage
from keras.models import load_model


# Keyboard controller
class Application(Frame):

    def __init__(self, kpub, master=None):

        Frame.__init__(self, master)
        self.pack()
        self.createWidgets()

        self.master.bind("<Left>", self.send_left)
        self.master.bind("<Right>", self.send_right)
        self.master.bind("<Up>", self.send_up)
        self.master.bind("<Down>", self.send_down)
        self.master.bind("<KeyRelease-Left>", self.left_release)
        self.master.bind("<KeyRelease-Right>", self.right_release)
        self.master.bind("<KeyRelease-Up>", self.up_release)
        self.master.bind("<KeyRelease-Down>", self.down_release)

        self.kpub = kpub


    def send_up(self, event=None):
        print "up"
        self.kpub.publish("up")
    def send_down(self, event=None):
        print "down"
        self.kpub.publish("down")
    def send_right(self, event=None):
        print "right"
        self.kpub.publish("right")
    def send_left(self, event=None):
        print "left"
        self.kpub.publish("left")
    def left_release(self, event=None):
        print "left released"
        self.kpub.publish("leftreleased")
    def right_release(self, event=None):
        print "right released"
        self.kpub.publish("rightreleased")
    def up_release(self, event=None):
        print "up released"
    def down_release(self, event=None):
        self.kpub.publish("upreleased")
        print "down released"
    def quit_safe(self, event=None):
        self.kpub.publish("downreleased")
        print("quitting safely")
        os.system('xset r on')
        self.quit()

    def createWidgets(self):
        self.QUIT = Button(self)
        self.QUIT["text"] = "QUIT"
        self.QUIT["fg"]   = "red"
        self.QUIT["command"] = self.quit_safe

        self.QUIT.pack({"side": "left"})

        self.up = Button(self)
        self.up["text"] = "up"
        self.up["fg"] = "blue"
        self.up["command"] = self.send_up
        self.up.pack({"side": "top"})

        self.down = Button(self)
        self.down["text"] = "down"
        self.down["fg"] = "blue"
        self.down["command"] = self.send_down
        self.down.pack({"side": "bottom"})

        self.right = Button(self)
        self.right["text"] = "right"
        self.right["fg"] = "blue"
        self.right["command"] = self.send_right
        self.right.pack({"side": "right"})

        self.left = Button(self)
        self.left["text"] = "left"
        self.left["fg"] = "blue"
        self.left["command"] = self.send_left
        self.left.pack({"side": "left"})


def callback_autopilot(data):
    global graph, model, commands, com_pub

    print('received image')
    np_arr = np.fromstring(data.data, np.uint8).reshape(1, 150, 250, 3)

    # tensorflow returns a bug if there is no graph...
    with graph.as_default():
        prediction = model.predict(np_arr)

    prediction = prediction[0]
    print('prediction : ', prediction)

    # TODO: only works if discrete...
    # gas
    predicted_gas = prediction[0]
    if predicted_gas > commands['go']:
        curr_gas = 'up'
    elif predicted_gas < commands['stop']:
        curr_gas = 'down'
    else:
        curr_gas = 'upreleased'

    # dir
    predicted_dir = prediction[1]
    if predicted_dir > commands['drive']:
        curr_dir = 'right'
    elif predicted_dir < commands['left']:
        curr_dir = 'left'
    else:
        curr_dir = 'leftreleased'

    com_pub.publish(curr_gas)
    com_pub.publish(curr_dir)

    print('current direction: ', curr_dir, ' current gas: ', curr_gas)


def main(controller):

    if controller == 'keyboard':

        os.system('xset r off')

        kpub = rospy.Publisher('dir_gas', String, queue_size=20)
        rospy.init_node('keyboard_pub', anonymous=True)

        print('Please control the car with the keyboard arrows')

        root = Tk()
        app = Application(kpub, master=root)
        app.mainloop()
        root.destroy()

    elif controller == 'gamepad':

        gp_gas_pub = rospy.Publisher('gas', Float32, queue_size=20)
        gp_dir_pub = rospy.Publisher('dir', Float32, queue_size=20)
        gp_rev_pub = rospy.Publisher('rev', Bool, queue_size=3)
        rospy.init_node('gp_pub', anonymous=True)

        print('Please control the car with the gamepad')

        quit = False
        while not quit:
            events = get_gamepad()
            for event in events:
                if event.ev_type == 'Absolute' and event.code == 'ABS_X':
                    if abs(event.state) > 9000:
                        gp_dir_pub.publish(event.state/32000.)
                    else:
                        gp_dir_pub.publish(0)
                if event.ev_type == 'Absolute' and event.code == 'ABS_Z':
                    if abs(event.state) > 10:
                        gp_gas_pub.publish(-1)
                if event.ev_type == 'Absolute' and event.code == 'ABS_RZ':
                    if abs(event.state) > 5:
                        gp_gas_pub.publish(event.state/255.)
                    else:
                        gp_gas_pub.publish(0)
                if event.code == 'BTN_MODE' and event.state == 1 :
                    print('Quitting, bye!')
                    quit = True
                if event.code == 'BTN_WEST' and event.state == 1:
                    print('Switching direction')
                    gp_rev_pub.publish(True)

    elif controller == 'autopilot':
        print('Autopilot is ready')

        rospy.init_node('autopilot', anonymous=True)
        com_pub = rospy.Publisher('dir_gas', String, queue_size=20)
        sub = rospy.Subscriber("/camera", CompressedImage, callback_autopilot, queue_size=1000)
        rospy.spin()

class ArgumentError(Exception):

    def __init__(self):
        pass

    def __str__(self):
        print('Error in arguments !')
        return self



if __name__ == '__main__':

    possible_arguments = ['-k', '--keyboard', '-g', '--gamepad',
                          '-a', '--autopilot', '-m', '--model',
                          '-c', '--commands_folder']
    arguments = sys.argv[1:]

    controller = 'keyboard'
    model_path = 'autopilot_2.hdf5'
    commands = {'go_t': 0.25, 'stop_t': -0.25, 'left_t': 0.5, 'right_t': -0.5,
                    'direction': 1, 'left': 310, 'right': 490, 'straight': 400,
                    'gas': 2, 'drive': 400, 'stop': 200, 'neutral': 360}

    print(arguments)


    i = 0
    print(i < len(arguments))
    while i < len(arguments):
        arg = arguments[i]
        if arg not in possible_arguments:
            raise ArgumentError
        if arg in ['-k', '--keyboard']:
            controller = 'keyboard'
        if arg in ['-g', '--gamepad']:
            controller = 'gamepad'
        if arg in ['-a', '--autopilot']:
            controller = 'autopilot'
        if arg in ['-m', '--model']:
            if i+1 >= len(arguments):
                raise ArgumentError
            model_path = arguments[i+1]
            if not os.path.isfile(model_path):
                print('This path does not exist : {}'.format(model_path))
                raise ArgumentError
            i += 1
        if arg in ['-c', '--commands-folder']:
            if i+1 >= len(arguments):
                raise ArgumentError
            command_file = arguments[i+1]
            if not os.isfile(command_file):
                print('No command_file found at : ', command_file)
                print('Using default thresholds')
            else:
                commands = load_commands(command_file)
                # TODO
            i += 1
        i += 1

    if controller == 'autopilot':
        model = load_model(model_path)
        graph = tf.get_default_graph()

    main(controller)
