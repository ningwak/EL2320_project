close all;
load('dataVictoriaPark.mat');

I = imread('vicPark.bmp','BMP');

% variables
t = 200; % overall time, maximum 61945
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
x(:, 1) = [-67.6493; -41.7142; 35.5*pi/180];
[u, z, x, m, tau] = initialize(controlSpeed, controlSteering, controlTime, ...
                                 laserData, laserTime, t, u, z, x, m, tau);
[omega, xi] = linearize(z, x, u, m, t, R, Q);
[omega_hat, xi_hat] = reduce(omega, xi, m, tau, x);
[mu, cov] = solve(omega_hat, xi_hat, omega, xi, tau, x, m);
%Plot(x,m,100);%the value of t_gps, maximum 4467, mapping between t and this value: 200-27 2000-180 3000-276 5000-336

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


for i = 1:1
    [z, m, tau] = check_correspondence(z, x, m, tau, omega, cov, Q, mu);
    [omega,xi] = linearize(z,x,u,m,t,R,Q);
    [omega_hat,xi_hat] = reduce(omega,xi,m,tau,x);
    [mu,cov] = solve(omega_hat,xi_hat,omega,xi,tau,x,m);
    
    %plot graph
    set(odoPath, 'xdata', x(1, :) + 67.6493, 'ydata', x(2, :) + 41.7142);
    set(obsFeature, 'xdata', m(1, :) + 67.6493, 'ydata', m(2, :) + 41.7142);
    drawnow;
    hold on;
end