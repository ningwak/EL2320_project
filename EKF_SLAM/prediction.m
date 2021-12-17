% This function performs the prediction step.
% Inputs:
%           mu(t-1)           (2N + 3) x 1   
%           sigma(t-1)        (2N + 3) x (2N + 3)
%           u(t)              odometry reading (r1, t, r2)
% Outputs:   
%           mu_bar(t)         (2N + 3) x 1 
%           sigma_bar(t)      (2N + 3) x (2N + 3)
function [mu_bar, sigma_bar] = prediction(mu, sigma, u)
    
    global R % covariance matrix of motion model | shape 3X3
    x = mu(1);
    y = mu(2);
    theta = mu(3);
    
    F = [eye(3) zeros(3, 2 * N)];
    vec_1 = 
    sigma_bar = G * sigma * G' + F' * R * F;
end