README.md

# Enabling Virtual CAN Interface for Launch Files

Some launch files require setting up virtual CAN (`vcan`) interfaces automatically.  
If any of these commands require `sudo`, the launch will fail because it cannot ask for a password interactively.

To avoid this, you must configure sudo so that your user can execute the required `vcan` commands **without a password**.

## 1. Create the sudoers rule

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

## 2. Execute ARUSSim with Control-RaspPi

A new launch was added in order to use and communicate with the Control-RaspPi repository instead of control_sim. IMPORTANT: Control-RaspPi MUST BE in a diferent workspace because of having other way to compile, having ARUSSim, DRIVERLESS2 and Control-RaspPi in the same ws will lead to innecesary compilations complications.

You will have to follow these steps:

1. Control-RaspPi compile:

```bash
cd ~/ws_raspi/src/Control-RaspPi/build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j"$(nproc)"
```

2. Launch ARUSSim with Control-RaspPi:

```bash
source ~/ws/install/setup.bash
ros2 launch arussim arussim_raspi_launch.py
```

Notes:
- The launch assumes `ws_raspi` is a sibling (is at the same level) of the ARUSSim workspace (e.g., `~/ws` and `~/ws_raspi`).
- To override the Control-RaspPi build directory, set the env var `CONTROL_RASPI_BUILD` before launching.

Example override:

```bash
export CONTROL_RASPI_BUILD="/path/to/ws_raspi/src/Control-RaspPi/build"
ros2 launch arussim arussim_raspi_launch.py
```

- The launch executes `sudo ./ControlRaspi` and creates `can0`, `can1` and `can2` as `vcan`.
