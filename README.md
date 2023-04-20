# r2auto_nav
ROS2 Code for EG2310, AY22/23 Sem2

This is Group 5's repository for EG2310 where the mission was to produce a can dispenser and a robot to autonomously deliver the can to desired tables in a restaurant.

### Code Breakdown

1. [r2auto_nav.py](r2auto_nav.py), [r2mover.py](r2mover.py), [r2moverotate.py](r2moverotate.py), [r2occupancy.py](r2occupancy.py), [r2occupancy2.py](r2occupancy2.py) and [r2scanner.py](r2scanner.py) are code that came from the intial fork of [shihchengyen's r2auto_nav repository](https://github.com/shihchengyen/r2auto_nav).
2. [map2base.py](map2base.py) contains transformations essential for accurate map data.
3. [waypoints.py](waypoints.py) is a script to save waypoints of the environment.
4. [r2table_nav.py](r2table_nav.py) is the main movement script controlling the entire delivery logic flow.
5. [esp32](esp32) folder contains arduino code for esp32 control.
6. [rpi](rpi) folder contains python scripts to publish data from sensors on the RaspberryPi to ROS topics.

Using this code assumes that your delivery robot and dispenser has a similar design to ours.

For more information on our delivery robot and dispenser designs, view our [documentation](documentation.pdf).


## Prerequisites

### Access to the following
1. Have access to a computer with Ubuntu 20.04 installed. A dual boot Linux is recommended but a virtual machine will suffice too. 
    - 1.1 For those with M series Macbook, you can follow [this tutorial](https://www.youtube.com/watch?v=suntoEurFio) to install Ubuntu onto your mac. Make sure to allocate 40GB. 35GB will barely scrape through.
2. Have access to a TurtleBot3 kit. 
    - 2.1 Have the ability to ssh into the TurtleBot3 RPi.
3. Have access to two ESP32s.
    - 3.1 Have the ability to flash code into the ESP32s through the Arduino IDE with ESP32 boards installed or an IDE of your choice.

### Software Installation Guide
1. Follow [the TurtleBot3 manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) **(Make sure to select Foxy.)** to install Ros2 and other relevant packages onto your linux computer. You can stop once you are able to teleoperate the turtlebot from your computer.
2. Follow [this guide](https://www.engineersgarage.com/raspberry-pi-esp32-esp8266-mqtt-iot/) to install and configure MQTT on your computer.

## Code Setup

### In your laptop:
1. Create a ROS2 package and clone this repository into that package using ```git clone https://github.com/dylkaw/r2auto_nav.git```. Make sure to edit the setup.py file to run the code.
2. Edit [r2table_nav.py](r2table_nav.py) with your own MQTT broker IP Address.
3. Build the package.

### In the RPi on the TurtleBot:
1. Copy [canPublisher.py](rpi/canPublisher.py) and [infraredPublisher.py](rpi/infraredPublisher.py) into the RPi on the turtlebot.

### On the ESP32s
1. Edit [servoControl.ino](esp32/servoControl.ino) with your own MQTT broker IP Address and wifi configurations.
2. Flash [servoControl.ino](esp32/servoControl.ino) into the ESP32 connected to the buttons and servo motor.
3. Flash [irEmitter.ino](esp32/irEmitter.ino) into the ESP32 connected to the IR Emitter.

## Mission Start

### Start-up (MUST DO BEFORE WAYPOINT CREATION AND NAVIGATION):
1. ssh into the Raspberry Pi on the Turtlebot.
2. Start the bringup from the RPi on the TurtleBot: ```roslaunch turtlebot3_bringup turtlebot3_robot.launch```
3. Start rslam from your laptop: 
  ``` ros2 launch turtlebot3_cartographer cartographer.launch.py ```
4. Start map2base publisher from your laptop:
``` ros2 run auto_nav map2base ```
5. This should always be done from the same point in your environment to ensure map accuracy.

### Waypoint Creation
1. Start waypoint creation script from your laptop:
``` ros2 run auto_nav waypoints ```
2. Teleoperate the delivery robot around the environment, entering 'w' to save your desired waypoints.
3. When done, enter 's' to save your waypoints in a `waypoints.pkl` file.

### Navigation
1. Start MQTT Broker on your computer by running ```sudo systemctl  start mosquitto```.
2. Ensure that delivery robot is not holding a can.
3. Start can publisher and infrared publisher from the RPi:
``` python canPublisher.py``` ``` python infraredPublisher.py ```
4. Ensure that your current working directory contains the `waypoints.pkl` file.
5. Start navigation script from your laptop:
``` ros2 run auto_nav r2table_nav ```
6. Press a button on your dispenser and watch the delivery robot go!

  

