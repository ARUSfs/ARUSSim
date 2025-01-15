# Track Editor
The `track_editor` is a tool for creating and editing Formula Student circuits.

## How to Start

1. Clone the repository and navigate to the `track_editor` folder.
2. Run `./main.py` to start the circuit editor application.

## How to use

You can create a circuit cone per cone or use the function `From points to circuit`. To use this function, you will have to:
1. Create the shape of the circuit with any cone (e.g., the gray ones), putting the cones in order.
2. Click on the flag icon.
3. Enter the requested data.
4. Add the time signals (the icon that looks like a camera). No more than 2.

A correct circuit must be:
- Clockwise 
- The main straight must be 6 meters long.
- TrackLength >= 200 and trackLength <= 500.
- MinTrackWidth >= 3.
- MinOuterDiameter >= 9.
- MaxLaneDistance <= 5.

## Folder Structure

- `main.py`: Main file that starts the circuit editor application.
- `drawView.py`: Script that handles the program's visualization.
- `guiLogic.py`: Contains the logic for the graphical user interface (GUI) of the circuit editor.
- `mapFile.py`: Manages the loading and saving of circuit map files.
- `points_to_circuit.py`: Contains functions to convert points into a circuit.

## Files

### `main.py`

This is the main file that starts the circuit editor application. Here you can find the functions that execute each button of the editor.

### `drawView.py`

Handles the visualization and drawing of the circuit in the graphical interface. This file is crucial for the visual representation of the circuit.

### `guiLogic.py`

Contains the logic for the graphical user interface. This file manages the user's interaction with the GUI and updates the circuit elements accordingly.

### `mapFile.py`

Manages the loading and saving of circuit map files. It allows the user to save their progress and load previously saved circuits.

### `points_to_circuit.py`

Contains a function that takes all the points through which our circuit will pass, translates and rotates them to position them at the origin (0,0), creates a curve with the points, calculates the tangents of each point and normalizes them, and places the internal and external cones of our circuit. Also it contains a PCD lector (circuit file lector) that makes a .json file with car's speed_profile from .pcd data