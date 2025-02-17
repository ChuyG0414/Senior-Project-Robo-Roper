clear, clc
data = readtable('data_collection.txt','Delimiter', ' ');
parsedData = cell(size(data.Var3));  % To store the parsed arrays
velocity = ones(1,length(data.Var3));
position = ones(1,length(data.Var3));
acceleration = ones(1,length(data.Var3));
voltage = ones(1,length(data.Var3));
time    = ones(1,length(data.Var3));
%second frequency begins at 320

for i = 1:height(data)
    % Extract the string from the current row of 'Var3'
    str      = data.Var3{i};
    splitValues = strsplit(str);  % Split by '→'
    parsedData{i} = str2double(splitValues);  % Store as a numeric array
end
for i = 1:length(parsedData)
    acceleration(i) = parsedData{i}(1);
    position(i)     = parsedData{i}(2);
    voltage(i)      = parsedData{i}(3);
end
time = data.Var1;
time = time - time(1);
time(320:end) = time(320:end) - time(320);

position_1 = position(1:319);
position_2 = position(320:end);

voltage_1  = voltage(1:319);
voltage_2  = voltage(320:end);

acceleration_1 = acceleration(1:319);
acceleration_2 = acceleration(320:end);

time_1 = time(1:319);
time_2 = time(320:end);

% Convert the duration array to seconds
time_seconds_1 = seconds(time_1);
n = length(time_seconds_1);
time_seconds_2 = seconds(time_2);
n_2 = length(time_seconds_2);
index_1 = [];
index_2 = []

% Calculate the time differences (dt) between successive samples
dt_1 = diff(time_seconds_1)'; % Time intervals (in seconds)
dt_2 = diff(time_seconds_2)';

% Calculate position differences (dx)
dx_1 = diff(position_1); % Position changes (in meters)
dx_2 = diff(position_2);

% Compute velocity as dx/dt
velocity_1 = dx_1 ./ dt_1; % Velocity (in meters per second)
v_1 = gradient(position_1,time_seconds_1);
velocity_2 = dx_2 ./dt_2;

for i = 1 : length(velocity_1)
    if ( isnan(velocity_1(i)) ) || (velocity_1(i) == inf)
        index_1 = [index_1, i];
    else
    end
end
velocity_1(index_1) = [];
acceleration_1(index_1) = [];
time_seconds_1(index_1) = [];
voltage_1(index_1) = [];
for i = 1 : length(velocity_2)
    if ( isnan(velocity_2(i)) ) || (velocity_2(i) == inf)
        index_2 = [index_2, i];
    else
    end
end
velocity_2(index_2) = [];
acceleration_2(index_2) = [];
time_seconds_2(index_2) = [];
voltage_2(index_2) = [];
%second frequency begins at 320
%% calculate model fo rvelocity using  least squares
x_history = [velocity_1,velocity_2];
u_history = [voltage_1(2:end), voltage_2(2:end)];
H = [x_history', u_history'];
m = [acceleration_1(2:end), acceleration_2(2:end)]';

P = (H' * H)^-1 * H' * m;
A_hat = P(1)
B_hat = P(2)



%%
% Define system dimensions (e.g., state and input sizes)
n = size(x, 1);  % State vector size
m = size(u, 1);  % Input vector size

% Initialize parameter estimates (A and B combined)
theta = zeros(n + m, 1);        % Initial parameter estimate vector [A; B]
P = 1000 * eye(n + m);          % Initial covariance matrix
lambda = 0.99;                  % Forgetting factor

% Loop through the data (new measurements of x and u)
for k = 1:length(y)
    % Construct input vector (state and control input)
    x_k = [x(:, k); u(k)];

    % Prediction (calculate predicted output)
    y_hat = theta' * x_k; 

    % Calculate the Kalman gain
    K = P * x_k / (lambda + x_k' * P * x_k);
    
    % Update the parameter estimates (A and B)
    theta = theta + K * (y(k) - y_hat);
    
    % Update the covariance matrix
    P = (P - K * x_k' * P) / lambda;
end









