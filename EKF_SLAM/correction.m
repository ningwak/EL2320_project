% This function performs the correction step.
% Inputs:
%           mu_bar(t)             (3N + 3) x 1   
%           sigma_bar(t)          (3N + 3) x (3N + 3)
%           landmarks_observed    1 x N   a vector representing which landmarks have been observed
% Outputs:   
%           mu(t)               (3N + 3) x 1 
%           sigma(t)            (3N + 3) x (3N + 3)
%           landmarks_observed  1 x N   a vector representing which landmarks have been observed
function [mu, sigma, landmarks_observed] = correction(mu_bar, sigma_bar, z, landmarks_observed)

%number of measurements in this timestep
n_measurements = size(z, 2);

% input all control info
for i = 1:n_measurements
end
end
    
