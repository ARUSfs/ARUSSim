#!/bin/bash

# Check if three or four parameters are provided
if [ "$#" -ne 2 ] && [ "$#" -ne 3 ] && [ "$#" -ne 4 ]; then
    echo "Usage: $0 <num_iterations> <event> <track (optional)> <laps (optional)>"
    echo "Events: Acceleration, Skidpad AutoX, Trackdrive, Custom"
    echo "Example: $0 8 Skidpad => Run 8 skidpads"
    echo "Example: $0 3 Custom FSG 2 => Run 3 iterations of the FSG track with 2 laps"
    echo "CSVs will be saved in the '~/ARUS_logs/csv' folder"
    exit 1
fi

NUM_ITERATIONS=$1
EVENT=$2
if [ "$#" -eq 2 ]; then
    # Set default track for Acceleration event
    if [ "$EVENT" == "Acceleration" ]; then
        TRACK="acceleration"
        LAPS=1
    # Assign arguments to variables
    # Set default track for Skidpad event
    elif [ "$EVENT" == "Skidpad" ]; then
        TRACK="skidpad"
        LAPS=4
    else 
        TRACK="FSG"
        LAPS=10
    fi
elif [ "$#" -eq 3 ]; then
    TRACK=$3
    LAPS=10
elif [ "$#" -eq 4 ]; then
    TRACK=$3
    LAPS=$4
fi

# Add '.0' to LAPS if it is an integer
if [[ "$LAPS" != *.* ]]; then
    LAPS="${LAPS}.0"
fi

# Validate that the number of iterations is a positive integer
if ! [[ "$NUM_ITERATIONS" =~ ^[0-9]+$ ]]; then
    echo "Error: num_iterations must be a positive integer."
    exit 1
fi

# Loop for the iterations
for ((i=1; i<=NUM_ITERATIONS; i++))
do
    ros2 launch arussim sim_automatic_launch.py track:=$TRACK event:=$EVENT laps_target:=$LAPS
    sleep 2
done
