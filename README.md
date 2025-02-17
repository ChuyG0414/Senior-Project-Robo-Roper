# Senior-Project-Robo-Roper
This is my senior project Robo-Roper. The project began this past summer and has since become my senior capstone project! I cannot go into too much detail due to legal reasons, but a high-level overview of the project includes developing an automated roping training system. This experience has been nothing but invaluable, it has given me the entire engineering process. Involving designing, simulation, testing, and reiteration. It has also humbled me extremely and made it known there is always room for improvement as an engineer! I learned a valuable lesson, communication! I quickly came to the realization I would require assistance specifically from a Mechanical Engineer because there were some problems I was having with the dynamics of the system. Had I not expressed my concern to my employer, progress would have significantly decreased. My responsibilities include programming, implementation of controls, sensor integration and simulation. Through these responsibilities I have practiced sourcing modules for correct configurations, reading data sheets, reading scholarly articles and effective communication between myself and my employer. I have attached some of the simulation files I created to show the project is feasible and can work. It utilizes simulation techniques such as data plotting, Runga Kutta (RK4), State Estimation, noise, Way point collection, PID controller, and even a velocity model was derived using Least Squares!

Description on each of the files:

ATV_Dynamics_3:
This file contains the dynamics or equation which governs how my system reacts. It contains equations describing the velocity of the vehicle, position of the linear actuator, position of the cart, and heading. 

ATV_Param: 
This file contains static values, such as mass of the cart, wheel base length, etc. It also contains gains used in the PID controllers.

ATV_dataPLotter_3:
This tool visualizes simulation data, including the cart's position, waypoints, velocity, steering angle, heading, and reference values. It provides a clear representation of the system’s actual response to commands. For example, if a velocity of 3 m/s is requested, the plot will display the cart accelerating gradually, reflecting real-world behavior without an instantaneous jump in speed.

atv_sim:
This is the simulation file, it brings all the other classes together. At the end of the simulation error in way point following is calculated and displayed, 

pathFollower: 
This is the path following algorthtim, I used the straight line following technique. 

pathManager_2:
This file controls parameters associated with the path follower such as, how aggressivley we would like to converge on a path. 

Gonzalez_HeadingController, Gonzalez_steeringAngleController,velocityController:
These files all have PID controllers implemented for controlling the heading, steering angle and velocity of the cart!

velocity_model:
This file imports previously collected acceleration and positional data. Using an accelerometer and a digital encoder I provided a sinusoid input to the carts throttle to properly charcaterize its acceleration. After data had been collected it imported to MATLAB, parsed and then Least Squares was applied to derive a velocity model.
