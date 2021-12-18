function [ G ]= jacobian_calculation(x, u, T)
    global vehicle
    a = vehicle.a;
    b = vehicle.b;
    L = vehicle.L;
    
    v = u(1);    % speed
    alpha = u(2); % steering angle
    theta = x(3); % heading angle
    
    G = [1 0 -T*v*(sin(theta) + 1/L*tan(alpha)*(a*cos(theta) - b*sin(theta))); 
         0 1  T*v*(cos(theta) - 1/L*tan(alpha)*(a*sin(theta) + b*cos(theta)));
         0 0  1];
end