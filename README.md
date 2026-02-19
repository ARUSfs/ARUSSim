# Enabling Virtual CAN Interface for Launch Files

Some launch files require setting up virtual CAN (`vcan`) interfaces automatically.  
If any of these commands require `sudo`, the launch will fail because it cannot ask for a password interactively.

To avoid this, you must configure sudo so that your user can execute the required `vcan` commands **without a password**.

## Create the sudoers rule

Run:

```bash
sudoedit /etc/sudoers.d/99-vcan
```
Paste the following content, and replace <your-username> with your actual Linux username:
```bash
Cmnd_Alias VCAN_CMDS = \
    /usr/sbin/modprobe vcan, \
    /usr/sbin/ip link add dev can0 type vcan, \
    /usr/sbin/ip link set up can0, \
    /usr/sbin/ip link set down can0, \
    /usr/sbin/ip link add dev can1 type vcan, \
    /usr/sbin/ip link set up can1, \
    /usr/sbin/ip link set down can1, \
    /usr/sbin/ip link add dev can2 type vcan, \
    /usr/sbin/ip link set up can2, \
    /usr/sbin/ip link set down can2

<your-username> ALL=(ALL) NOPASSWD: VCAN_CMDS
```

# Configuration of Control-RaspPi 

In order to communicate via vcan and use the Control-RaspPi repository, you must specify the relative path from the "[control_raspi.cpp](src/arussim/src/control_raspi.cpp)" file to the Control-RaspPi executable. 

Control-RaspPi acts as a interface to handle events suchs as executing/killing the process, resolving the path to the executable, log errors caused by Control-RaspPi, ...

**IMPORTANT: READ BELOW**
You need 2 workspaces (preferably parents) since Control-RaspPi doesn't use colcon, therefore it can't be compiled as the same way as ARUSSim.

By default, the relative path is set to "../../../../../ws_raspi/src/Control-RaspPi/build/ControlRaspi". This assuming that workspaces for both ARUSSim and Control-RaspPi are parents and your ws for Control-RaspPi is named "ws_raspi", feel free to chenge it as long as you change the relative path.

## Control-RaspPi behaviour on ARUSSim

Because of "Stop Simulation" button not having a topic, control_raspi.cpp only kills Control-RaspPi execution by pressing the "Reset Simulation" button so please, press both of them.

Also, if you notice the car behaving anormally, close the simulation and restart it (or press the "reset" button on the left down corner). Maybe you didn't restarted the simulation properlly and there are 2 Control-RaspPi process running (not ideal).
