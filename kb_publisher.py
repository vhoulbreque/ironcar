import rospy
from std_msgs.msg import String
from Tkinter import *
import sys

print("please control the car with the keyboard arrows")
kpub = rospy.Publisher('dir_gas', String, queue_size=20)
rospy.init_node('keyboard_pub', anonymous=True)


class Application(Frame):

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



    def createWidgets(self):
        self.QUIT = Button(self)
        self.QUIT["text"] = "QUIT"
        self.QUIT["fg"]   = "red"
        self.QUIT["command"] = self.quit

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


root = Tk()

app = Application(master=root)
app.mainloop()
root.destroy()


