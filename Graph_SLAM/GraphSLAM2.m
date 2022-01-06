close all;
load('dataVictoriaPark.mat');
load('aa3_gpsx.mat');

I = imread('vicPark.bmp','BMP');

% variables
t = 2000; % overall time, maximum 61945
u = zeros(2, t - 1); % control signal 2 * (t-1) [speed; steering angle]
z = []; % measurement 5 * unknown
        % [direction; distance; radius of tree; time index; correspondence]
x = zeros(3, t); % estimated state of car 3 * t [x; y; phi]
m = []; % landmarks 3 * unknown [x; y; r]
tau = []; % time moments that each landmark is observed 

% define noise
R = diag([0.05 0.05 0.1]); % (x, y, th)  [0.05 0.05 0.001]
Q = diag([1 0.01 1]);   % (range, angle, signature)     [1 0.01]

% initialize figure
figure('name','SLAM Demo','color','w','units','normalized',...
         'outerposition',[0 0 1 1]);
hold on;box on; axis equal;
image([-142 188],[260 -120],I,'alphadata',1);  % Satelite map
axis([-120 160 -50 240]); 
%plot(Lo_m(1:7:end,1)+75,La_m(1:7:end,1)+45, 'b.', 'markersize', 5);  % Ground truth
plot(Lo_m(1:7:end,1)+67.6493,La_m(1:7:end,1)+41.7142, 'b.', 'markersize', 5);  % Ground truth
hold on;
set(gca,'xtick',[],'ytick',[]); 
set(gca,'Color','w','XColor','w','YColor','w')
odoPath     = plot(0,0,'r-','linewidth',1.5,'erasemode','normal');
obsFeature  = plot(0,0,'y+','linewidth',1.5,'markersize',8,'erasemode','normal');

truth_       = plot(0,0,'b.','linewidth',3,'markersize',8,'erasemode','normal');
odoPath_     = plot(0,0,'r-','linewidth',1.5,'erasemode','normal');
obsFeature_  = plot(0,0,'y+','linewidth',1.5,'markersize',8,'erasemode','normal');
label        = [odoPath_,truth_, obsFeature_];
lgd          = legend(label,'Estimated Path','GPS','Features');
set(lgd,'box','on','position',[0.61 0.83 0.1 0.1],'color','w',...
    'FontSize',11,'fontname','song','fontweight','bold');

% initialize
tic;
x(:, 1) = [0; 0; 0]; %[-67.6493; -41.7142; 35.5*pi/180];
[u, z, x, m, tau] = initialize(controlSpeed, controlSteering, controlTime, ...
                                 laserData, laserTime, t, u, z, x, m, tau);
[omega, xi] = linearize(z, x, u, m, t, R, Q);
[omega_hat, xi_hat] = reduce(omega, xi, m, tau, x);
[mu, cov] = solve(omega_hat, xi_hat, omega, xi, tau, x, m);
    
for i = 1:1
    mu = reshape(mu, 3, []);
    x = mu(:, 1:t);
    m = mu(:, t+1:end);
    mu = reshape(mu, [], 1);
    
    [z, m, tau] = check_correspondence(z, x, m, tau, omega, cov, Q, mu);
    [omega,xi] = linearize(z,x,u,m,t,R,Q);
    [omega_hat,xi_hat] = reduce(omega,xi,m,tau,x);
    [mu,cov] = solve(omega_hat,xi_hat,omega,xi,tau,x,m);
end
time = toc;

%mu = reshape(mu, 3, []);
%x = mu(:, 1:t);
%m = mu(:, t+1:end);
    
%plot graph
set(odoPath, 'xdata', x(1, :), 'ydata', x(2, :));
set(obsFeature, 'xdata', m(1, :), 'ydata', m(2, :));
drawnow;
hold on;

%rotate x and m
theta = (35.5 + 180) * pi / 180;
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
for i = 1:size(x, 2)
    rotatePoint = R * [x(1, i), x(2, i)]';
    x(1, i) = rotatePoint(1);
    x(2, i) = rotatePoint(2);
end
theta = 35.5 * pi / 180;
R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
x(3, :) = x(3, :) + theta;
for i = 1:size(m, 2)
    rotatePoint = R * [m(1, i), m(2, i)]';
    m(1, i) = rotatePoint(1);
    m(2, i) = rotatePoint(2);
end

%plot graph
set(odoPath, 'xdata', x(1, :), 'ydata', x(2, :));
set(obsFeature, 'xdata', m(1, :), 'ydata', m(2, :));
drawnow;
hold on;

% plot error
gps_t = 180;
pose_errors = zeros(2, gps_t);
mu_ptr = 1;
for i = 1:gps_t
    while controlTime(mu_ptr) < timeGps(i)
        mu_ptr = mu_ptr + 1;
    end
    if mu_ptr > t
        gps_t = i - 1;
        break;
    end
    pose_errors(1, i) = abs(x(1, mu_ptr) - Lo_m(i) - 67.6493);
    pose_errors(2, i) = abs(x(2, mu_ptr) - La_m(i) - 41.7142);
end

timesteps = 1:gps_t;
mex = mean(pose_errors(1, 1:gps_t));
mey = mean(pose_errors(2, 1:gps_t));

figure('Name', 'Evolution State Estimation Errors');
subplot(2,1,1);
plot(timesteps, pose_errors(1,1:gps_t));
ylabel('error\_x [m]');
title(sprintf('error on x, mean error = %.2fm', mex));
subplot(2,1,2);
plot(timesteps, pose_errors(2,1:gps_t));
ylabel('error\_y [m]');
title(sprintf('error on y, mean error = %.2fm', mey));

%mapping between t and gps time: 200-27 2000-180 3000-276 5000-336