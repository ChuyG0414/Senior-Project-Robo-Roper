%parameter file for linear actuator, heading and velocity PID control
%simulation parameters

P.t_plot = 0.1; % sample rate for plotter and animation
P.t_start = 0;
P.t_s = .01;
P.t_end = 60; %30 seconds


%system parameters for linear actuator, load on linear actuator used for calculaiting max possible speeds.

%animation parameters 
P.motor_width = 6;
P.motor_height = 3;
P.actuator_width = 2;
%initial conditions
P.load = 50;            %pounds of load
P.z_initial = 0;        %intital position of linear actuator, how much it is extruded out
P.phi_initial = 0;

P.r = 0.085725; %3.375 inches, distance between linear actuator and wheel pivot point  in meters

%controller limits, will be using a -100 for full reverse, 100 for full forward, 0 for stop

P.lower_limit = -100;
P.upper_limit = 100;



P.t_r_z = .001;    %rise time for z postion of linear actuator keep in mind needs to be about 10 times faster than outer rise time 
P.zeta_z = .707;     %damping coefficent of z
P.t_r_z_integrator = 0; %rise time for z integrator
%frequency cut off
frequency_LA = 5;        %frequency in hertz we want to update cart wheels about 5 times per second
P.sigma_LA = 1/(2*pi*frequency_LA);

%end Linear Actuator system parametrs 



%system parameters for heading
P.v     = 6.25856;    %velocity, in meters/s this is equivilent to 14 mph
P.ell   = 1.6;                  %distance been center of wheels in front to the back in meters
P.chi_0 = 0;        %intial heading
P.x_0   = 0;
P.y_0   = 0;
P.phi_0 = 0;
P.phi_max_speed = 0.06626606; %max speed of linear atcuator in m/s


%system limits
P.theta_upper_limit = 100;
P.theta_lower_limit = -100;
P.steering_max_angle = deg2rad(41);
%freqeuncy cut off
frequency_heading = 10;
P.sigma_heading = 1/(2*pi*frequency_heading);


%gains
%velocity tuning gains
P.heading_threshold_for_deceleration = deg2rad(15);
P.v_max = 4.2;
P.v_0 = 0;
P.v_cutoff_frequency = .1; 
P.v_A_hat  = -.4438;
P.v_B_hat  = .3778;
P.v_min    = 0;
P.e_in_min = 0;
P.e_in_max = 5;
P.v_t_r    = 1.25;
P.v_zeta   = .707;
P.v_t_r_integrator = .98;
%phi tuning gains, rise time and zeta/damping coefficent
P.ki_phi = 450;
P.kp_phi = 1650;
P.kd_phi = 0;
%heading gains
P.t_r_theta = 2.5;
P.zeta_theta = .707;
P.kd_theta = 75;
P.kp_theta = 245;
P.ki_theta = 1;


%path managment and following parameters
P.north = 0;
P.east = 0;
P.down = 0;

P.W = load("data.csv"); %fist component is east, 2nd is north, 3rd is up compnent
%P.W = [0 10 0; 0 20 0; -5 25 0; -10 30 0 ; -20 40 0 ; 0 0 0]';
% 
P.chi_inf = pi/2;
P.k_path = .25;
P.k_orbit = 1;

