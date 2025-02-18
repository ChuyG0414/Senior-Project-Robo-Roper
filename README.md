# Senior-Project-Robo-Roper
This is my senior project Robo-Roper. The project began this past summer and has since become my senior capstone project! I cannot go into too much detail due to legal reasons, but a high-level overview of the project includes developing an automated roping training system. This experience has been nothing but invaluable; it has given me the entire engineering process. It involving designing, simulation, testing, and reiteration. It has also extremely humbled me and made it apparent there is always room for improvement as an engineer. I learned a valuable lesson, communication. I quickly came to the realization I would require assistance specifically from a Mechanical Engineer because there were some problems I had with the dynamics of the system. If I had not expressed my concern to my employer, progress would have significantly decreased. My responsibilities include programming, implementation of controls, sensor integration and simulation. Through these responsibilities I have practiced sourcing modules for correct configurations, reading data sheets, reading scholarly articles and effective communication between myself and my employer. I have attached some of the simulation files I created to show the project is feasible and can work. It utilizes simulation techniques such as data plotting, Runga Kutta (RK4), State Estimation, noise, Way point collection, PID controller, and even a velocity model was derived using Least Squares!

I am currently working on the physical and c++ implementation portion of this project I will update this repository once this portion is completed.
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

Way_point_generation:
Tool to select valid waypoints used in the path following of the cart. 

atvSensors2:
This is the sensor configuration portion of the project in a C-based Arduino file. It sets up the encoder, accelerometer, magnetometer, GPS, and PCNT peripheral on the ESP32.

Link1: 
https://youtu.be/pjGH6eBTWfI

This link is a video of the above files in action! It showcases the simulation with out electrical noise to make it easier to interpret the plots.
The 2x1 plot, titled "Voltage to Cart Speed Controller" and "Effort of LA", represents the two system inputs. LA stands for Linear Actuator.

The "Voltage to Cart Speed Controller" plot shows the input controlling the vehicle’s speed. The speed controller operates on a 0–5 V signal, where 5 V corresponds to full throttle and 0 V indicates no acceleration.

The "Effort of LA" plot measures the intensity of the Linear Actuator’s movement. A value of 100 commands the actuator to extend at maximum speed, while -100 commands it to retract at maximum speed.

The 3x1 Plot Explanation:
The 3x1 plot, titled "Velocity," "Heading," and "Steering Angle," displays the current state of each parameter compared to their commanded values. Each plot shows how closely the system follows the desired targets for speed, direction, and steering input.

The figure titled "Position", displays the waypoints in green, representing the target path, while the blue line shows the vehicle’s actual path as it attempts to follow those waypoints.

The figure titled "Error in Waypoint Following" shows the deviation, measured in meters, between the vehicle's actual position and the intended waypoint. 

Link2:
https://youtu.be/5jL4DsDddaE

This link is a simulation similar to Link1 but it includes electrical noise to showcase performance of the controller still exists despite there being noise. It also walks through the way point generation to show how the waypoints are collected.
