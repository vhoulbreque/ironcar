import rospy
from std_msgs.msg import String, Float32
from Tkinter import *
import sys, os

os.system('xset r off')

print("please control the car with the keyboard arrows")
gas_pub = rospy.Publisher('gas', Float32, queue_size=20)
#pic_pub = rospy.Publisher('pic', String, queue_size=20)
rospy.init_node('keyboard_pub', anonymous=True)


class Application(Frame):

    def send_up(self, event=None):
        print "up"
        gas_pub.publish(0.2)
        #pic_pub.publish("up")
    def send_down(self, event=None):
        print "down"
        gas_pub.publish(-0.2)
        #pic_pub.publish("down")
    def up_release(self, event=None):
        print "up released"
        gas_pub.publish(0)
        #pic_pub.publish("upreleased")
    def down_release(self, event=None):
        print "down released"
        gas_pub.publish(0)
        #pic_pub.publish("downreleased")
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

    def __init__(self, master=None):

        Frame.__init__(self, master)
        self.pack()
        self.createWidgets()

        self.master.bind("<Up>", self.send_up)
        self.master.bind("<Down>", self.send_down)
        self.master.bind("<KeyRelease-Up>", self.up_release)
        self.master.bind("<KeyRelease-Down>", self.down_release)


root = Tk()

app = Application(master=root)
app.mainloop()
root.destroy()



