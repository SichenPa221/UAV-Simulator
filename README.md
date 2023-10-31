# UVA-Simulator

# System description: 

A flying robot is considered as a simple point translating in the environment (3 DOF). This robot has a maximum velocity but we suppose that an infinite acceleration is possible and the system has no delay. The environment imposes no constraint, so robot movements in the world perfectly reflect the command. At start, the robot is at a predefined initial position in the environment and it does not move.

# Objective: 

Implement a simulation of this system that:
- at program launch time, allows the user to set the initial position of the robot in the world, its maximum velocity and the time step of the simulation.

- If the robot is static, it waits an input of the user. User can then either :
	- input a target position where the robot has to go to. The robot then starts moving from its current position to that target position.
	- ask for exiting the program. The program immediately ends.

- If the robot moves it prints its current position (X, Y, Z) in the terminal (at every iteration). Once it has reached the target position, the robot stops and alerts the user with a message in the terminal.

# Additional information:
- provide the source code and the Makefile (or CMake project file) used to build the code. 
- the program must be written in C/C++ for a linux environment.
- write a program as clean as possible: it must be well structured, simple to understand and bug free.
- for interaction with the user, use only terminal standard input/output (no graphical interfaces).
- do not use third party libraries except c/c++ standard libraries.
