# A quadcopter obstacle avoidance simulator
This repo contains the C code for a small simulator of a quadcopter, whose purpose is to avoid obstacles with a laser scanner while reaching the goal position.
The software uses POSIX thread to implement concurrency, GNU Scientific Library for computation of dynamics and Allegro 4.4 for the graphics.
## Dependencies
- Pthread
- Allegro 4.x
- [tomlc99](https://github.com/cktan/tomlc99)
## Installation and Execution
Compile with `make`.
Run with `sudo ./main`.
## Usage
Select the target waypoints with left mouse click. Cancel the waypoint with the right mouse click. 
The other editable variables have the dedicated keys shown in the top right part of the interface.
Exit the simulation with [ESC].
