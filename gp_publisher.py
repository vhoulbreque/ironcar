import time
import rospy
from inputs import get_gamepad
from std_msgs.msg import Float32, Bool

gp_gas_pub = rospy.Publisher('gas', Float32, queue_size=20)
gp_dir_pub = rospy.Publisher('dir', Float32, queue_size=20)
gp_rev_pub = rospy.Publisher('rev', Bool, queue_size=3)
rospy.init_node('gp_pub', anonymous=True)

quit = False
print("Starting loop")
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
            #print(event.state)
            if abs(event.state) > 5:
                gp_gas_pub.publish(event.state/255.)
            else:
                gp_gas_pub.publish(0)
        if event.code == 'BTN_MODE' and event.state == 1 :
            print("Quitting, bye!")
            quit = True
        if event.code == 'BTN_WEST' and event.state == 1:
            print("Switching direction")
            gp_rev_pub.publish(True)
