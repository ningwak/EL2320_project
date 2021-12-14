load('dataVictoriaPark.mat');

% variables
t = 200; % overall time, maximum 61945
u = zeros(2, t - 1); % control signal 2 * (t-1) [speed; steering angle]
z = []; % measurement 5 * unknown
        % [direction; distance; radius of tree; time index; correspondence]
x0 = zeros(3, t); % initial estimated state 3 * t [x; y; phi]
x = zeros(3, t); % estimated state 3 * t [x; y; phi]
m = []; % landmarks 3 * unknown [x; y; r]
mObs = []; % time moments that each landmark is observed 
omega = []; % information matrix
xi = []; % information vector

% initialize
x0(:, 1) = [0; 0; 0];
[u, z, x0, m, mObs] = initialize(controlSpeed, controlSteering, controlTime, ...
                                 laserData, laserTime, t, u, z, x0, m, mObs);

