clc,close all, clear
v_linear_actuator = 0.06626606; %meters/sec
max_z             =v_linear_actuator/2;
r                 = 0.085725;       %distance between linear actuator and center of rotation
v_cart            = 6.25856/2;      %meters/sec
max_dist          = v_cart;   %sample taken twice every seconds    
max_degrees       = asin(max_z/r);  %max possible turning radius based off of velcity of linear actuator

%initiate way points

waypoints = [ones([2,60]);zeros([1,60])];
waypoints(1:2,1) = [0;0];
index = 2;
%create circle
x_c = waypoints(1,1);                        %x coordinate center of circle
y_c = waypoints(2,1);                        %y coordinate center of circle
theta = linspace(pi/2 - max_degrees,pi/2 + max_degrees , 100);
x = max_dist * cos(theta) + x_c;
y = max_dist * sin(theta) + y_c;



figure(1)
plot(x,y, waypoints(1,1),waypoints(2,1),'g--o');
axis([-20 20 -20 50]);
grid on;

% figure(2)
% plot(waypoints(1,:),waypoints(2,:),'g--o');

while waypoints(1,1) ~= waypoints(1,index) && waypoints(2,1) ~= waypoints(2,index)
    figure(1);
    [x_click, y_click] = ginput(1); % Get a clicked point
    dist_origin        = sqrt((x_click - waypoints(1,1))^2 + (y_click - waypoints(2,1))^2);    
    
    % Find the closest point on the line to the clicked point
    [~, idx] = min((x - x_click).^2 + (y - y_click).^2); % Find the closest point
    x_closest = x(idx);
    y_closest = y(idx);
    dist_arc = sqrt( (x_click - x_closest)^2 + (y_click - y_closest)^2 );

    if dist_origin < dist_arc
        waypoints(1:2,index ) = waypoints(1:2,1);
        writematrix(waypoints(1:3,1:index),'data.csv')
        break
    else
        waypoints(1:2,index) = [x_closest;y_closest];
        heading_vector = waypoints(1:2,index) - waypoints(1:2, index-1);
        heading = atan2(heading_vector(2),heading_vector(1));
        x_c = waypoints(1,index);                        %x coordinate center of circle
        y_c = waypoints(2,index);                        %y coordinate center of circle
        theta = linspace(heading - max_degrees,heading + max_degrees , 100);
        x = max_dist * cos(theta) + x_c;
        y = max_dist * sin(theta) + y_c;
        figure(1);
        plot(x,y, waypoints(1,1:index),waypoints(2,1:index),'g--o');
        axis([-30 30 -30 60]);
        grid on;
    end

    index = index + 1;
end