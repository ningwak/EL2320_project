close all;
load('dataVictoriaPark.mat');

% variables
t = 1000; % overall time, maximum 61945
u = zeros(2, t - 1); % control signal 2 * (t-1) [speed; steering angle]
z = []; % measurement 5 * unknown
        % [direction; distance; radius of tree; time index; correspondence]
x = zeros(3, t); % estimated state of car 3 * t [x; y; phi]
m = []; % landmarks 3 * unknown [x; y; r]
tau = []; % time moments that each landmark is observed 

%define noise
R = diag([0.05 0.05 0.001]); % (x, y, th)  [0.05 0.05 0.001]
Q = diag([1 0.01 1]);   % (range, angle, signature)     [1 0.01]

% initialize
x(:, 1) = [0; 0; 35.5*pi/180];
[u, z, x, m, tau] = initialize(controlSpeed, controlSteering, controlTime, ...
                                 laserData, laserTime, t, u, z, x, m, tau);
[omega, xi] = linearize(z, x, u, m, t, R, Q);
[omega_hat, xi_hat] = reduce(omega, xi, m, tau, x);
[mu, cov] = solve(omega_hat, xi_hat, omega, xi, tau, x, m);

for i = 1:5
    [z, m, tau] = check_correspondence(z, x, m, tau, omega, cov, Q, mu);
    [omega,xi] = linearize(z,x,u,m,t,R,Q);
    [omega_hat,xi_hat] = reduce(omega,xi,m,tau,x);
    [mu,cov] = solve(omega_hat,xi_hat,omega,xi,tau,x,m);
end

%plot graph
Plot(x,m,100);%110 is the value of t_gps. you can change the value between 1 and 4467  200-27 2000-180