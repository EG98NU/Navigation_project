% Initialize parameters for the simulation

% 40.61043876911262, 9.756915512868472
height= 0;                      % height of the center of the navigation frame
latitude= 40.61043876911262;    % latitude of the center of the navigation frame
longitude = 9.756915512868472;  % longitude of the center of the navigation frame

% Magnetic field vector [nT]
B_n = wrldmagm(height,latitude,longitude,decyear(2020,7,4),'2020');
B_n = B_n*1e-3;      %conversion from nT to microT
g = gravitywgs84(height,latitude);  % gravity acceleration 
g_n = [ 0 0 g ]';     % gravity acceleration vector in navigation frame

% Analog Devices ADXL354 Accelerometer parameters
Var_acc = (g*12e-6)^2*(1/ST);  % accelerometer variance of white noise [(m/s^2)^2]

% Bosch BMG250 Gyroscope parameters
Var_gyro = (0.014*pi/180)^2*(1/ST);     % gyroscope variance of white noise [(rad/s)^2]
bias_gyro = 0.5*(pi/180)*[1 1 1]';      % gyroscope bias [rad/s]

% Honeywell HMC5883L Magnetometer parameters
f_mag = 50;                  % magnetometer frequency [Hz]
Var_mag = (40e-3)^2*f_mag;   % magnetometer variance of white noise [(microT)^2]

% Decawave DW1000 UWB sensor parameters
% Points = sf*[ 0   0  0;
%              70   2  5;
%              72  50 -5;
%              68 100  0;
%               2 102  5;
%              -2  48 -5]; % Reference points from where the distances are measured

Points = sf*[ 0   0  0;
             70   2  1;
             72  50  0;
             68 100  1;
              2 102  0;
             -2  48  1]; % Reference points from where the distances are measured

Var_uwb = 0.1^2; % UWB sensor ariance of white noise [m^2]

% Initial conditions
pos_0 = tmd(1,2:4)';    % initial position
vel_0 = [0 0 0]';    % initial velocity filter
vel_0_t = vel_n(1,:)';    % initial velocity filter
quat_0 = quat_c(1,:)';  % initial attitude

% initialize P for attitude filter
Cov_quat = (1*pi/180)^2;
Cov_omega_bias = (1*pi/180)^2;
P0_att=[Cov_quat*eye(4), zeros(4,3); zeros(3,4), Cov_omega_bias*eye(3)];

% initialize P for position & velocity filter
Cov_pos = 1;
Cov_vel = 0.5;
P0_pv = diag([Cov_pos Cov_pos Cov_pos Cov_vel Cov_vel Cov_vel]);


