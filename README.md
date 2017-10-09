# Particle Filter

## Description

This program implements a particle filter in order to determine the location of a moving vehicle inside a simulator. You
can find a link to the simulator [here](https://github.com/udacity/self-driving-car-sim/releases).

The program uses an initial set of noisy GPS coordinates to generate N number of particles stochastically distributed 
around those coordinates. The program then comapres a detailed landmark map, car control inputs, and sensor measurement 
observations to determine which particle most closley represents our vehicle.

This project was completed as part of the UDACITY Self-Driving Car Engineer Nanodegree Program. For more information, or 
to enroll today, please check [here.](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013) 
 
## Main.cpp

This file handles the communication between the simulator and the program. This file also calls init, predict, and update
functions.

## Particle-filter.cpp

This file contains the functions necissary to perform particle filtering

### init()
This function takes in GPS coordinates, and creates Num_particles number of particles random-normally distributed around those coordinates.
The particles are assign an id, x, y, bearing, and weight values. A weight array is also initialized (the weight array allows for easy
checking of partcile weights, and is associated with each particle by index)


### Prediction()

This function takes in controls information -velocity, yaw rate, and time- and performs updates each particles position 
based on these values. Noise is introduced for each particles final position and bearing based on control nois values.


### updateWeights()

This function contains most of the processing. An array of observations and map landmarks are fed as input to the function.
For each particle, the observation data is applied to our particle's position+bearing. The particle observations are then 
compared to our map landmarks, and the closest landmark to each observation is determined. Finally, a Multi Variate Gaussian
Probability Density Function performed on each landmark-observation pair, and the results are multiplied together.

The resultant product is assigned as the weight of the particle.


### resample()

This function creates a discrete distribution based off the weight of each particle, which means the higher the weight, 
the more discrete elements will be assigend to that particle.

A random generator is then used with the discrete distribution to select Num_particles particles to be selected to continue forward with. 
The selected particles are statistically likely to be of higher weight than the particles selected from. After many particle
filtering steps, only very likely candidates for our vehicles position (should) be left.



## Dependencies / Installation
This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.


## Running the Code
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and intall uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./particle_filter

Alternatively some scripts have been included to streamline this process, these can be leveraged by executing the following in the top directory of the project:

1. ./clean.sh
2. ./build.sh
3. ./run.sh

Note that the programs that need to be written to accomplish the project are src/particle_filter.cpp, and particle_filter.h

The program main.cpp has already been filled out, but feel free to modify it.

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator.

INPUT: values provided by the simulator to the c++ program

// sense noisy position data from the simulator

["sense_x"] 

["sense_y"] 

["sense_theta"] 

// get the previous velocity and yaw rate to predict the particle's transitioned state

["previous_velocity"]

["previous_yawrate"]

// receive noisy observation data from the simulator, in a respective list of x/y values

["sense_observations_x"] 

["sense_observations_y"] 


OUTPUT: values provided by the c++ program to the simulator

// best particle values used for calculating the error evaluation

["best_particle_x"]

["best_particle_y"]

["best_particle_theta"] 

//Optional message data used for debugging particle's sensing and associations

// for respective (x,y) sensed positions ID label 

["best_particle_associations"]

// for respective (x,y) sensed positions

["best_particle_sense_x"] <= list of sensed x positions

["best_particle_sense_y"] <= list of sensed y positions


## Inputs to the Particle Filter
You can find the inputs to the particle filter in the `data` directory. 

#### The Map*
`map_data.txt` includes the position of landmarks (in meters) on an arbitrary Cartesian coordinate system. Each row has three columns
1. x position
2. y position
3. landmark id

### All other data the simulator provides, such as observations and controls.

> * Map data provided by 3D Mapping Solutions GmbH.


