### ROS Sphero Bolts
`demiurge-sphero-bolt` is a ROS meta-package for controlling a swarm of Sphero Bolt robots. This package provides the necessary software to operate the Sphero Bolt robots and connect them to a central computer via Raspberry Pi devices.

#### Requirements

To use demiurge-sphero-bolt, you will need the following:

Ubuntu 20.04 Focal Fossa installed on the central computer and each Raspberry Pi device.

ROS Noetic installed on the central computer and each Raspberry Pi device.

Python libraries: [`spherov2.py`](https://github.com/poiuytrezaur/spherov2.py.git), bleak

### Installation & Setup 

1.	Installing Ubuntu 20.04 Focal Fossa on the PC and each raspberry pi:
    Detailed instructions and troubleshooting on: https://phoenixnap.com/kb/install-ubuntu-20-04 
    For the Raspberry Pi, you can use the Raspberry Pi Imager software to flash the SD card.
    Detailed instructions and troubleshooting on: https://www.yodeck.com/docs/how-to-articles/how-to-flash-an-sd-card-using-pi-imager/
	
2.	Installing Python 3.x:
  •	Open a terminal window on the Ubuntu computer
  •	Run the following command to update the package list: ```sudo apt-get update```
  •	Run the following command to install Python 3.x: ```sudo apt-get install python3```


3.	Installing ROS Noetic:
•	Open a terminal window on the Ubuntu computer
•	Follow the instructions on the official ROS Noetic installation guide: http://wiki.ros.org/noetic/Installation/Ubuntu

4.	 Installing this package:
•	Clone this project repository containing the controller software and the driver.
•	Open a terminal window and navigate to the spherov2.py folder in the package directory 
•	Run the following command to get the submodule spherov2.py : git submodule update --init --recursive
•	Install the spherov2 dependencies, specifically bleak with: ```pip install bleak```

5.	Setting Up the Raspberry Pi:
•	Install Ubuntu 20.04 Focal Fossa on each Raspberry Pi (following the steps in subsections 1)
•	Install Python 3.x and ROS Noetic on each Raspberry Pi (following the steps in subsections 2 and 3)
•	Clone the driver and controller software (following the steps in subsection 4)

### Usage 

1)	Launch the control software on the central computer. This can be done by following these steps: 
	-	Open 6 terminal windows on the central computer.
	-	In the first terminal window, launch the ROS master node by entering the following command: ```roscore```
	-	In the second terminal window, launch the multi_sphero_control.launch file located in the sphero_bolt_controller/launch directory, by entering the following command: ```roslaunch sphero_bolt_controller multi_sphero_control.launch```  
	Note: Any lines in the multi_sphero_control.launch file corresponding to unused robots should be commented out to prevent unnecessary errors.
	-	In the third terminal window, launch the arena by navigating to the directory where the MoCA arena package was downloaded and built
	-	In the remaining four terminal windows, connect to each Raspberry Pi device via SSH by entering the following command (assuming "raspberry1" is the hostname used when saving the static IP address of the 		   Raspberry Pi device): ```ssh raspberry1```. Enter the password that was defined during the installation process. If a different hostname was used, substitute it for "raspberry1" in the command, or use the                 following syntax to connect using the IP address directly: ```ssh <username>@<raspberry_ip_address>```
2)	In each Raspberry Pi terminal window, navigate to the sphero_bolt_driver/launch directory within the project folder and execute the multi_sphero_driver.launch file by entering the following command: ```roslaunch sphero_bolt_driver multi_sphero_driver.launch```

Once these steps are complete, the Sphero Bolt robots should be connected and waiting for a first shake to switch from Idle mode to random walking mode.

### Contributors

David Garzón Ramos <david.garzon.ramos@ulb.be>

Florian Noussa Yao <florian.noussa.yao@ulb.be>

# License
   This Project is released under the MIT License. See the MIT (LICENSE) file for more details.
