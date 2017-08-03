# IRONCAR NODE

Google doc with tutorial https://docs.google.com/document/d/1jyRhlbmthMA_DuuulYnzUT38okIF_KFZH0a4hh8NCg8/edit?usp=sharing .

## SETUP

### Keras install 

### Requirements
Pip requirements can be found in requirements.txt.

The node packages are to be installed using package.json.


### Troubleshooting

## Launching
In order to communicate between the car, the laptop and another device like a smartphone,
we used socketio on python and nodejs.Therefore, we need to launch a server node on the raspi
and clients on other devices. A python client on the raspi ensure the communication with the 
car hardware (motors, camera  and other potential sensors if you want to add any).


### On the raspberry pi
Two programs are to be launched on the raspi. You can use screen if you don't want to carry 
a laptop. Those two programs can be run as daemon if you really don't want to use a laptop. 

* server node: `node car_server.js` This will launch the node server. By default, it runs on 
the raspi localhost on the port 8000. 
* python client: `python ironcar_master.py` This will launch the python client that directly
controls the car and takes commmands from the node server.

### On the laptop
You can Launch controller.py on the laptop (don't forget to change the ip to put your raspi ip)
if you have a gamepad to control the car in training/ direction auto. You might need to change
this script to adapt to your gamepad. As an example, we used a xbox gamepad and listened to the
left joystick for direction and RT trigger for the gas, LT to break. 

### On any device
The user interface is a javascript client that can be launched in any browser in theory 
(chrome and safari have been tested). Just go to YOUR_RASPI_IP:8000 and you should be able to 
choose the mode, the model, the speed, and control the car with a keyboard. 
 

## Details



