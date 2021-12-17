clear;
clc;
load('dataVictoriaPark.mat');

% Global variables
global vehicle x0 noise 
vehicle.L = 2.83;
vehicle.a = 0.95;
vehicle.b = 0.5;
vehicle.H = 0.76;
x0 = [-67.6493; -41.7142; 35.5*pi/180];

noise.R = diag([0.05 0.05 0.001]); % (x, y, th)  [0.05 0.05 0.001]
noise.Q = diag([1 0.01 1]);   % (range, angle, signature)     [1 0.01]

% % variables
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

INF = 1000;
N = size(m, 2);
% The number of landmarks in the map

landmarks_observed = false(1, N);
% landmarks_observed is a vector representing which landmarks have been observed

sigma_robot = zeros(3);
% Covariance of robot pose

sigma_robot_map = zeros(3, 3 * N);
% Covariance between robot pose and map

sigma_map = INF * eye(3 * N);
% Covariance of the map

sigma = [[sigma_robot sigma_robot_map]; [sigma_robot_map' sigma_map]];
mu_bar = zeros(t - 1, 3 * N + 3, 1);
sigma_bar = zeros(t - 1, 3 * N + 3, 3 * N + 3);

% initialize
x0(:, 1) = [0; 0; 0];
[u, z, x0, m, mObs] = initialize(controlSpeed, controlSteering, controlTime, ...
                                 laserData, laserTime, t, u, z, x0, m, mObs);
                             
mu = x0;
% mu is a (3N + 3) x 1 vector representing the mean of the normal distribution

N_t = 0;
% Map size at the moment
for i = 1:t - 1
    [mu_bar(t), sigma_bar(t)] = prediction(mu, sigma, u(:, i));
    % Perform the EKF prediction step
% 	observation = data_association(mu, sigma, z(t));
%     % Obtain current observation using the data association
%     [mu, sigma, landmarks_observed] = correction(mu_bar, sigma_bar, observation, landmarks_observed);
%     % Perform the EKF correction step
end