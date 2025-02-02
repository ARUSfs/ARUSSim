
# Automatic Simulations Node

The **Automatic Simulations** node is designed to manage and execute automated simulations within the ARUSSim simulator. It handles simulation parameters, monitors lap counts, and shutsdown the simulation upon completion of the target laps.

## Features

- **Event**: Adjusts target laps based on the selected event (e.g., Trackdrive, Acceleration).
- **Real-time Monitoring**: Subscribes to lap time updates and tracks the number of completed laps.
- **Logging**: Provides informative logs about the simulation status and parameters.
- **Shutdown**: Automatically terminates the simulation once the target number of laps is reached.


## Launch

Launch the Automatic Simulations launch with the following command:

```bash
ros2 launch arussim sim_automatic_launch.py
```

## Bash

Also you can use bash file `automatic_sim.sh`:

```bash
./automatic_sim.sh <num_iterations> <event> <track (optional)> <laps (optional)>
```

Replace `<parameters>` with a valid value:

- **num_iterations**: Integer. The number of iterations to perform.
- **event**: String. The type of event (Trackdrive, AutoX, Acceleration, or Skidpad). If you want a custom simulation, input *Custom*
- **track**: (Optional) String. The name of the track. Input a logic track, dont use FSG circuit for skidpad event.
- **laps**: (Optional) Integer. The number of laps of each simulation.

