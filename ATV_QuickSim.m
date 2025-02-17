%%
clc, close all, clear
ATV_Param
% instantiate system, controller, and reference classes
system = Gonzalez_LADynamics(P);        %inputs = (load, initial z position, sample rate)
controller_steering = Gonzalez_SteeringAngleController(P);
controller_heading = Gonzalez_HeadingController(P);
reference = signalGenerator(deg2rad(45),.001,0);
disturbance = signalGenerator(1.0, 0.0);
noise = signalGenerator(0.001);
% instantiate the data plots and animation
dataPlot = Gonzalez_LAdataPlotter(P);
%animation = Gonzalez_LAanimation(3);
% main simulation loop
t = P.t_start; % time starts at t_start
%y = system.h(); % system output at start of simulation
states = [P.z_initial;0];
vehicle_states = [P.x_0;P.y_0;P.phi_0;P.chi_0];
while t < P.t_end
% set time for next plot
    t_next_plot = t + P.t_plot;
     % updates control and dynamics at faster simulation rate
    while t < t_next_plot
        r = reference.square(t); % assign reference
        d = 0;%disturbance.step(t); % simulate input disturbance
        n = noise.random(t); % simulate noise
        phi_ref = controller_heading.update(vehicle_states(4),r);
        u = controller_steering.update(phi_ref, states + n); % update controller
        [states,vehicle_states] = system.update(u+d,0); % Propagate the dynamics
        t = t + P.t_s; % advance time by Ts
    end
 % update animation and data plots
    %animation.update(states(1));
    
    dataPlot.update(t, r,states,vehicle_states, u);
end


%%
clc, close all, clear
ATV_Param
% instantiate system, controller, and reference classes
system = Gonzalez_LADynamics(P);        %inputs = (load, initial z position, sample rate)
controller_steering = Gonzalez_SteeringAngleController(P);
controller_heading = Gonzalez_HeadingController(P);
reference = signalGenerator(deg2rad(30),.0001,0);
disturbance = signalGenerator(1.0, 0.0);
noise = signalGenerator(0.01);

%intialize path manager and path follower
pathMan = pathManager(P);

pathFollow = pathFollower(P);
% instantiate the data plots and animation
dataPlot = Gonzalez_LAdataPlotter(P);


%animation = Gonzalez_LAanimation(3);
% main simulation loop
t = P.t_start; % time starts at t_start
%y = system.h(); % system output at start of simulation
states = [P.z_initial;0];
vehicle_states = [P.x_0;P.y_0;P.phi_0;P.chi_0];
while t < P.t_end
% set time for next plot
    t_next_plot = t + P.t_plot;
     % updates control and dynamics at faster simulation rate
    while t < t_next_plot
        [y_man, pathCompleted] = pathMan.update(vehicle_states);
        [v_r,chi_command]      = pathFollow.update(vehicle_states,y_man,vehicle_states(4));
        %r = reference.square(t); % assign reference
        d = 0;%disturbance.step(t); % simulate input disturbance
        n = noise.random(t); % simulate noise
        phi_ref = controller_heading.update(vehicle_states(4),chi_command);         %problem lies in steering angle calculation
        u = controller_steering.update(phi_ref, states + n); % update controller
        [states,vehicle_states] = system.update(u+d,pathCompleted); % Propagate the dynamics
        t = t + P.t_s; % advance time by Ts
    end
 % update animation and data plots
    %animation.update(states(1));
    
    dataPlot.update(t, chi_command,states,vehicle_states, u);
    dataPlot.addWaypoints();
end
% dataPlot.finalPlot();

%% test for steering controller 
clc, close all, clear
ATV_Param
% instantiate system, controller, and reference classes
system = steering_dynamics(P);        %inputs = (load, initial z position, sample rate)
controller_steering = Gonzalez_SteeringAngleController(P);
controller_heading = Gonzalez_HeadingController(P);
reference = signalGenerator(deg2rad(41),.05,0);
disturbance = signalGenerator(1.0, 0.0);
noise = signalGenerator(0.01);
% instantiate the data plots and animation
dataPlot = steering_dataPlotter(P);
%animation = Gonzalez_LAanimation(3);
% main simulation loop
t = P.t_start; % time starts at t_start

linear_actuator_states = [P.z_initial;P.phi_0; 0;0];
while t < P.t_end
% set time for next plot
    t_next_plot = t + P.t_plot;
     % updates control and dynamics at faster simulation rate
    while t < t_next_plot
        r = reference.square(t); % assign reference
        d =0;%disturbance.step(t); % simulate input disturbance
        n = noise.random(t); % simulate noise
        linear_actuator_states(1) = linear_actuator_states(1) ;%+ n;
        u = controller_steering.update(r, linear_actuator_states); % update controller
        [linear_actuator_states] = system.update(u+d); % Propagate the dynamics
        t = t + P.t_s; % advance time by Ts
    end
 % update animation and data plots
    %animation.update(states(1));
    
    dataPlot.update(t, r,linear_actuator_states, u);
end

%% test for heading dynamics 
clc, close all, clear
ATV_Param
% instantiate system, controller, and reference classes
system = ATV_Dynamics(P);        %inputs = (load, initial z position, sample rate)
controller_steering = Gonzalez_SteeringAngleController(P);
controller_heading = Gonzalez_HeadingController(P);
reference = signalGenerator(deg2rad(-50),.0001,0);
disturbance = signalGenerator(1.0, 0.0);
noise = signalGenerator(0.01);

% instantiate the data plots and animation
dataPlot = heading_dataPlotter(P);


%animation = Gonzalez_LAanimation(3);
% main simulation loop
t = P.t_start; % time starts at t_start
%y = system.h(); % system output at start of simulation
linear_actuator_states = [P.z_initial;0];               %position of linear act
vehicle_states = [P.v;P.chi_0;P.phi_0];
position       = [P.x_0; P.y_0];
while t < P.t_end
% set time for next plot
    t_next_plot = t + P.t_plot;
     % updates control and dynamics at faster simulation rate
    while t < t_next_plot
        n = noise.random(t); % simulate noise

        %postion, velocity, heading, steering angle
        chi_command = reference.square(t);

        position       = system.position;
        vehicle_states = system.states;
        z              = system.previous_z;
        
        %calculate steeirng angle and adjust heading
        phi_ref        = controller_heading.update(vehicle_states(2),chi_command);         %problem lies in steering angle calculation
        u = controller_steering.update(phi_ref, z); % update controller
        
        %update dynamis
        u_v_phi = [5;u];
        system.update(u_v_phi ); 


        t = t + P.t_s; % advance time by Ts
    end
 % update animation and data plots
    %animation.update(states(1));
    
    dataPlot.update(t, chi_command,position,vehicle_states, u_v_phi);
end
%% test way point following and path manager 
clc, close all, clear
ATV_Param
% instantiate system, controller, and reference classes
system = ATV_Dynamics_rungakutta(P);        %inputs = (load, initial z position, sample rate)
controller_steering = Gonzalez_SteeringAngleController(P);
controller_heading = Gonzalez_HeadingController(P);
reference = signalGenerator(deg2rad(30),.0001,0);
disturbance = signalGenerator(1.0, 0.0);
noise = signalGenerator(0.01);

%intialize path manager and path follower
pathMan = pathManager(P);

pathFollow = pathFollower(P);
% instantiate the data plots and animation
dataPlot = ATV_dataPlotter_2(P);


%animation = Gonzalez_LAanimation(3);
% main simulation loop
t = P.t_start; % time starts at t_start
%y = system.h(); % system output at start of simulation
states = [P.z_initial;0];
vehicle_states = [P.v;P.chi_0;P.phi_0;0];
position       = [P.x_0; P.y_0];
pathCompleted = 0;
while t < P.t_end && pathCompleted == 0
% set time for next plot
    t_next_plot = t + P.t_plot;
     % updates control and dynamics at faster simulation rate
    while t < t_next_plot
        %pathmanger and path follower, gives me chi commanded

        [y_man, pathCompleted] = pathMan.update(position);
        [v_r,chi_command]      = pathFollow.update(position,y_man,vehicle_states(2));
        
        n = noise.random(t); % simulate noise

        %postion, velocity, heading, steering angle, position of linear actuator

        position       = system.position;
        vehicle_states = system.states + n;
        vehicle_states_dot = system.states_dot;
        z              = system.previous_z;

        %currently checking why phi is being restricted to 10 degrees hypothesis its due to dynamics
        throw_vehicle_states = [vehicle_states;vehicle_states_dot];

        
        %calculate steeirng angle and adjust heading
        phi_ref        = controller_heading.update(throw_vehicle_states,chi_command);         %problem lies in steering angle calculation
        u = controller_steering.update(phi_ref, z); % update controller
        
        %update dynamis
        u_v_phi = [5;u];
        system.update(u_v_phi,pathCompleted ); 


        t = t + P.t_s; % advance time by Ts
    end
 % update animation and data plots
    %animation.update(states(1));
    
    dataPlot.update(t, chi_command,position,vehicle_states, u);
end
closest_path_point = nan([2 length(P.W)]);
position = [dataPlot.x_history;dataPlot.y_history];
for i = 1: length(P.W)
    [~, idx] = min( (position(1,:) - P.W(1,i) ).^2 + ( position(2,:) - P.W(2,i) ).^2);
    closest_path_point(1:2,i) = [position(1,idx);position(2,idx)];
end
e_path = (closest_path_point - P.W(1:2,:));
e_magnitude = sqrt(sum(e_path.^2));

Waypoints = P.W(1:2,:);
i = linspace(1,length(Waypoints),length(Waypoints));
colors = zeros(length(Waypoints), 3);  
for j = 1:length(Waypoints)
    if e_magnitude(j) > 1.524
        colors(j,:) = [1, 0, 0];
    else
        colors(j,:) = [0,1,0];
    end
end
figure(4);

scatter(i,e_magnitude,100,colors,"filled");
title("Error in Waypoint Following");
xlabel("Waypoint(n)");
ylabel("Error(m)");
