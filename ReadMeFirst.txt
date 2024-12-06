Howdy,
In all of the files shown in the main branch with the message "Initial Automation Commit", the MainFile.py is the most important one. 
It is the closest file to complete out of all of them. The only reason the rest are there is due to different ideas being shown and explored.
The files VisDet.py and MainFile.py are fully documented, the rest are less important and more esoteric.

In order to turn the robot on, you must:
1. Plug in to wall
2. Flip the switch on the top of the Power Converter
3. Release the E-Stop
4. Reset
5. Press the power button on the back, and wait for it to click.

To turn it off:
1. Press the E-Stop
2. Flip the switch on top of the Power Converter
3. Wait for the lights to die
4. Unplug from wall

In order to connect to the robot, you will need to connect the ethernet cable to your laptop, or raspberry pi, and set up an ethernet connection to it.
IPv4:
Address: 192.168.0.101
Subnet Mask: 255.255.255.0
Gateway: 192.168.0.100

To connect to the web portal, plug the robot in to your computer and go to a search bar and type the Gateway, after the robot is turned on.

When you receive an error due to the robot, there is a chance you will have to either:
1. Connect to the mecademic web portal, and reset the error there
2. Use the command in the mecademic library to reset the error.
