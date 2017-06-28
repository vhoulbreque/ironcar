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
import numpy as np
import tensorflow as tf
import roslib
import scipy.misc

from Tkinter import *
from inputs import get_gamepad
from std_msgs.msg import String, Float32, Bool
from PIL import Image
from sensor_msgs.msg import CompressedImage
from keras.models import load_model
from utils import ArgumentError, load_controls


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
        print("up")
        self.kpub.publish("up")
    def send_down(self, event=None):
        print("down")
        self.kpub.publish("down")
    def send_right(self, event=None):
        print("right")
        self.kpub.publish("right")
    def send_left(self, event=None):
        print("left")
        self.kpub.publish("left")
    def left_release(self, event=None):
        print("left released")
        self.kpub.publish("leftreleased")
    def right_release(self, event=None):
        self.kpub.publish("rightreleased")
        print("right released")
    def up_release(self, event=None):
        self.kpub.publish("upreleased")
        print("up released")
    def down_release(self, event=None):
        self.kpub.publish("downreleased")
        print("down released")
    def quit_safe(self, event=None):
        print("quitting safely")
        os.system('xset r on')  # to enable 'key continuously pressed' back
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
    global graph, model, controls, gas_pub, dir_pub

    img_n, img_height, img_width, img_channel = 1, 150, 250, 3

    np_arr = np.fromstring(data.data, np.uint8).reshape(img_n, img_height, img_width, img_channel)
    np_arr = np_arr[:, 80:, :, :]  # resize of the image happens after !

    acc_arr = np.array([np.array(list(map(float, data.header.frame_id.split('_')[:-1])))])

    # tensorflow returns a bug if there is no graph...
    with graph.as_default():
        prediction = model.predict([np_arr, acc_arr])

    prediction = prediction[0]

    print('prediction : ', prediction)

    prediction = list(prediction)
    index_class = prediction.index(max(prediction))
    curr_dir = -1 + 2 * float(index_class)/float(len(prediction)-1)

    curr_gas = 0

    gas_pub.publish(curr_gas)
    dir_pub.publish(curr_dir)

    print('current direction: ', curr_dir, ' current gas: ', curr_gas)


def callback_log(data):
    global graph, model, controls, gas_pub, dir_pub, save_folder_laptop, n_img

    img_n, img_height, img_width, img_channel = 1, 150, 250, 3
    im_arr = np.fromstring(data.data, np.uint8).reshape(img_n, img_height, img_width, img_channel)
    save_arr = np.array(im_arr[0,:,:,:], copy=True)
    im_arr = im_arr[:, 80:, :, :]  # resize of the image happens after !
    print('im_arr.shape : ', im_arr.shape)

    acc_arr = np.array([np.array(list(map(float, data.header.frame_id.split('_')[:-1])))])

    # tensorflow returns a bug if there is no graph...
    with graph.as_default():
        prediction = model.predict([im_arr, acc_arr])

    prediction = prediction[0]

    print('prediction : ', prediction)

    prediction = list(prediction)
    index_class = prediction.index(max(prediction))
    curr_dir = -1 + 2 * float(index_class)/float(len(prediction)-1)
    curr_gas = 0

    # TODO: Include xacc, yacc and zacc too
    image_name = os.path.join(save_folder_laptop, 'frame_'+ str(n_img) +
                                '_gas_' + str(curr_gas) + '_dir_' +
                                str(curr_dir) + '.jpg')
    scipy.misc.imsave(image_name, save_arr)
    n_img += 1
    print('image saved at path : {}'.format(image_name))


def main(controller):

    global gas_pub, dir_pub

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
        gas_pub = rospy.Publisher('gas', Float32, queue_size=20)
        dir_pub = rospy.Publisher('dir', Float32, queue_size=20)
        sub = rospy.Subscriber("/camera", CompressedImage, callback_autopilot, queue_size=1000)
        log_image = rospy.Subscriber('/camera', CompressedImage, callback_log, queue_size=1000)
        rospy.spin()


if __name__ == '__main__':

    possible_arguments = ['-k', '--keyboard',
                          '-g', '--gamepad',
                          '-a', '--autopilot',
                          '-m', '--model',
                          '-c', '--controls_folder',
                          '-v', '--verbose']
    arguments = sys.argv[1:]

    controller = 'keyboard'
    controls = {'go_t': 0.25, 'stop_t': -0.25, 'left_t': 0.5, 'right_t': -0.5,
                    'direction': 1, 'left': 310, 'right': 490, 'straight': 400,
                    'gas': 2, 'drive': 400, 'stop': 200, 'neutral': 360}

    models_folder = 'models'
    model_path = os.path.join(models_folder, 'autopilot_2.hdf5')

    save_folder_laptop = 'log_info'
    if not os.path.exists(save_folder_laptop):
        os.makedirs(save_folder_laptop)

    print(arguments)

    n_img = 0


    i = 0
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
            model_path = os.path.join(models_folder, arguments[i+1])
            if not os.path.isfile(model_path):
                print('This path does not exist : {}'.format(model_path))
                raise ArgumentError
            i += 1
        if arg in ['-c', '--controls-file']:
            if i+1 >= len(arguments):
                raise ArgumentError
            controls_file = arguments[i+1]
            if not os.isfile(controls_file):
                print('No controls_file found at : ', controls_file)
                print('Using default thresholds')
            else:
                controls = load_controls(controls_file)
            i += 1
        i += 1

    if controller == 'autopilot':
        print('Loading model at path : ', model_path)
        model = load_model(model_path)
        graph = tf.get_default_graph()
        print('Finishing loading model at path : ', model_path)

    main(controller)
