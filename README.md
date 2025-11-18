# equilibrio
A ball balancing project

![](docs/balance.mov)

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

# Hardware needed:
* Electronics:
    * Adafruit Metro M4 board (later to be a custom board)
    * Custom shield with headers for servo motors, touchscreen sensor, and UART header for console I/O
    * CMSIS-DAP programmer - I'm using Atmel-ICE
    * 2 MG995 180 degree metal gear servo motors
    * 2 Aluminum 25T Servo Horn Steering Arms
    * 2 RC Aluminum Linkages for Servo
    * 1 VSDISPLAY 8.4" 4 wire resistive touchscreen
* Custom balance table hardware
    * 3d-printed table hardware
    * Wooden base board to mount the table hardware

# Zephyr commands needed:
* Install the zephyr ecosystem for your OS following https://docs.zephyrproject.org/latest/develop/getting_started/index.html
* Build: need to be in the embedded_src directory
    > west build -b metro_m4 -p always -- -DBOARD_ROOT=.
    - the -DBOARD_ROOT=. tells west to look for the custom board I created in the current directory (which has the boards subdirectory)
* Flash:
    > west flash

# Notes about design choices:
* Servo:
    * I started with DS-S006L servo’s with plastic gears and plastic steering arms. These seem to work fine, but there is a lot of play in the coupling. Also, the steering arms are not very durable and split with the screws provided
    * I changed out MG995 servo’s with metal gears and steering arms. Hopefully this will reduce some of the play

* Sensor:
    * I'm using a VSDISPLAY 8.4" (172mm x 129.5mm touch area) 4-wire resistive Touch Panel I found on Amazon. I’m looking for larger (and square shaped) panels, but they get expensive quickly with increasing size
        * How 
    * When using an non-square panel, the sensor gains are different in the x and y directions. This is because the voltage drop is across a different distance in each direction, but the ADC range is the same. Ex:
        * 12b ADC = 4096 counts
        * x → 164mm/4096counts = 0.04004mm/count
        * y → 105mm/4096counts = 0x02563mm/count
    * MCU Pin voltage driving the panel lower than 3.3V:
        * The panel resistance is in the range of 300-700 ohms depending on the direction. Typical code examples for Arduino and other platforms drive this sensor with GPIO pins. This is a too much current draw for a typical GPIO pin, so the pin output voltage will drop. You can typically increase the drive strength of a GPIO, but make sure it is enough drive the load or you will measure lower voltages than expected. Ex:
            * typical default drive strength of a MCU GPIO is about 2mA
                * Max load → r=v/i = 1,650 ohms
                * at 300 ohms and 3.3V, → i=v/r = 11mA !!!
                * output pin voltage will droop to where it can actually drive this load
                * v = ir = 0.6V
    * Seeing a strange offset in the sensor reading when no touch is present:
        * The panel some capacitance between the two layers of ITO, and the ADC has some sample capacitance (2-3pF). When you switch on voltage across one layer while the other layer is floating (high impedance into your ADC) and no touch is present, this sensed line voltage will jump to about ½ of the driven voltage and then very slowly decay back to 0. This is because, with no touch, there is very high (almost infinite) resistance between in the sensed line because it’s floating. This causes a large RC time constant. If you sample the line before this decays, you will read the floating voltage instead of the actual voltages
        * My quick solution was to add some pull-down resistance to the sampled lines of the touch panel. I used 5k because I had these in my stock. This does cause a small voltage drop in the reading (but can be accounted for) and significantly speeds up the decay of this voltage, effectively removing the offset from the reading

* Balance table:
    * the play in the servo coupling was accentuated by my initial table design which had the fixed table mount in the middle of the sensing table. This puts the servo load in both tension and compression depending on the position of the ball. If the ball is between the fixed table mount and the servo, the linkage is in compression. If the ball is past the fixed table mount from the servo, the linkage is in tension. When it switches between tension and compression, the play in the linkage causes some error in the actuation of the table. I changed my design to put a fixed table mount on one side of the table, and then put the servo linkage connections at the two opposite corners of the table. This causes the linkage to always be in compression, removing the "transition play"

