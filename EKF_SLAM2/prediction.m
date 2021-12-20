% This function performs the prediction step.
% Inputs:
%           mu(t-1)           (3N + 3) x 1   
%           sigma(t-1)        (3N + 3) x (3N + 3)
%           u(t)              odometry reading (speed, steering angle, time)
% Outputs:   
%           mu_bar(t)         (3N + 3) x 1 
%           sigma_bar(t)      (3N + 3) x (3N + 3)
function [mu_bar, sigma_bar] = prediction(mu, sigma, u)
    
    global noise % covariance matrix of motion model | shape 3X3
    global vehicle % the distance between wheel axles
    a = vehicle.a;
    b = vehicle.b;
    L = vehicle.L;
    
    v = u(1);    % speed
    alpha = u(2); % steering angle
    delta_t = 0.025; % moving time
    
    x = mu(1);
    y = mu(2);
    theta = mu(3);
    
    m1 = a*sin(theta) + b*cos(theta);
    m2 = a*cos(theta) - b*sin(theta);
    
    F = [eye(3) zeros(3, size(sigma, 2) - 3)];
    mu_bar = mu + delta_t * F' * [v*(cos(theta) - 1/L*tan(alpha)*m1); 
                             v*(sin(theta) + 1/L*tan(alpha)*m2);
                             v/L*tan(alpha)];
    Gx = jacobian_calculation(mu ,u, delta_t);
    G = eye(size(sigma, 1)) + F' * (Gx - eye(3)) * F;
    sigma_bar = G * sigma * G' + F' * noise.R * F;
    mu_bar(3) = mod(mu_bar(3), 2 * pi);
end