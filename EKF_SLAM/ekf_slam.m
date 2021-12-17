close all;
clear all;
clc;

load aa3_dr.mat
load aa3_gpsx.mat
load aa3_lsr2.mat

landmarks
% Load landmarks
sensor_data
% Load data

N = size(landmarks, 2);
% The number of landmarks in the map

landmarks_observed = false(1, N);
% landmarks_observed is a vector representing which landmarks have been observed

mu = zeros(2 * N + 3, 1);
% mu is a (2N + 3) x 1 vector representing the mean of the normal distribution

sigma_robot = zeros(3);
% Covariance of robot pose

sigma_robot_map = zeros(3, 2 * N);
% Covariance between robot pose and map

sigma_map = INF * eye(2 * N);
% Covariance of the map

sigma = [sigma_robot sigma_robot_map];
for t = 1:80
    [mu_bar, sigma_bar] = prediction(mu, sigma, data.timestep(t).odometry);
    % Perform the EKF prediction step
    
end


