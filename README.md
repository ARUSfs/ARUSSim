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
    /usr/sbin/ip link set down can1

<your-username> ALL=(ALL) NOPASSWD: VCAN_CMDS
```
