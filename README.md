# IronCar
## Summary
The IronCar project is made for those who need an easy setup for an autonomous vehicle like an RC car using a raspberrypy. 
It was made by the team from XBrain.
We created two branches: 
* a ROS based project on the master branch. We originally started with ROS as it is an easy framework to setup such robotic systems that require asynchronous commands.
* a nodejs based project on the ironcar_node branch. We implemented this to make it easier for people to install and run everything. It is more straightforward to setup and run on a raspberry pi for example.

## Setup

* requirements to be updated
* Either ROS or nodejs is needed on the raspi. For training with a gamepad, you will need it on a laptop as well
* Keras is needed on the machine where you want to run the prediction. Both a laptop or a raspi can run the prediction
