load('dataVictoriaPark.mat');

% variables
t = 200; % overall time, maximum 61945
u = zeros(2, t - 1); % control signal 2 * (t-1) [speed; steering angle]
z = []; % measurement 5 * unknown
        % [direction; distance; radius of tree; time index; correspondence]
x = zeros(3, t); % estimated state of car 3 * t [x; y; phi]
m = []; % landmarks 3 * unknown [x; y; r]
tao = []; % time moments that each landmark is observed 

%define noise
R = diag([0.05 0.05 0.001]); % (x, y, th)  [0.05 0.05 0.001]
Q = diag([1 0.01 1]);   % (range, angle, signature)     [1 0.01]

% initialize
x(:, 1) = [0; 0; 0];
[u, z, x, m, tao] = initialize(controlSpeed, controlSteering, controlTime, ...
                                 laserData, laserTime, t, u, z, x, m, tao);

[omega,xi] = linearize(z,x,u,m,t,R,Q);

[omega_hat,xi_hat] = reduce(omega,xi,m,tao,x);

[x,m,cov] = solve(omega_hat,xi_hat,omega,xi,tao,x,m);