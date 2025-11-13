# equilibrio
A ball balancing project

# Goals:
This is a learning project focused on several learning goals:
* State Space control theory
    * I will start with a simple PID controller, but move to state space control
* Zephyr OS
    * Investigate the west build system 
    * Create a custom board within Zephyr
    * 
* Cloud Connectivity
    * send ball position and other meta data to the cloud
    * Change the balls balancing location from the cloud
    * AWS IoT for the first iteration

# Future goals:
In the future, the following technologies may be added for learning:
* BLE
* Display/graphics
* Capacitive Touch may be investigated as a sensor for ball position

# Block diagram:
![image](docs/BlockDiagram.png)

# Firmware Architecture:

# Zephyr commands needed:
* Build:
    > west build -b metro_m4 --pristine -- -DBOARD_ROOT=.
    - the -DBOARD_ROOT=. tells west to look for the custom board I created in the current directory


