import os
import time
import sys
import rospy

from Tkinter import *
from inputs import get_gamepad
from std_msgs.msg import String, Float32, Bool


# Keyboard controller
class Application(Frame):

    def __init__(self, master=None):

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


    def send_up(self, event=None):
        print "up"
        kpub.publish("up")
    def send_down(self, event=None):
        print "down"
        kpub.publish("down")
    def send_right(self, event=None):
        print "right"
        kpub.publish("right")
    def send_left(self, event=None):
        print "left"
        kpub.publish("left")
    def left_release(self, event=None):
        print "left released"
        kpub.publish("leftreleased")
    def right_release(self, event=None):
        print "right released"
        kpub.publish("rightreleased")
    def up_release(self, event=None):
        print "up released"
        kpub.publish("upreleased")
    def down_release(self, event=None):
        print "down released"
        kpub.publish("downreleased")
    def quit_safe(self, event=None):
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


def main(controller):

    if controller == 'keyboard':

        os.system('xset r off')

        kpub = rospy.Publisher('dir_gas', String, queue_size=20)
        rospy.init_node('keyboard_pub', anonymous=True)

        print('Please control the car with the keyboard arrows')

        root = Tk()
        app = Application(master=root)
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


if __name__ == '__main__':

    possible_arguments = ['-k', '--keyboard', '-g', '--gamepad']
    arguments = sys.argv[1:]

    controller = 'keyboard'

    i = 0
    while i < len(arguments):
        arg = arguments[i]
        if arg not in possible_arguments:
            raise ArgumentError
        if arg in ['-k', '--keyboard']:
            controller = 'keyboard'
        if arg in ['-g', '--gamepad']:
            controller = 'gamepad'
        i += 1

    main(controller)
