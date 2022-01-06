clear;
clc;
load('dataVictoriaPark.mat');

I = imread('vicPark.bmp','BMP');
load aa3_gpsx ; 
% plot(Lo_m(1:275),La_m(1:275),'.') ;
% plot(Lo_m(1:151),La_m(1:151),'.') ;
% hold on;

global vehicle x0 noise N_t
vehicle.L = 2.83;
vehicle.a = 0.95;
vehicle.b = 0.5;
vehicle.H = 0.76;
N_t = 0;
x0 = [-67.6493; -41.7142; 35.5*pi/180]; % initial estimated state 3 * t [x; y; phi]
%x0 = [-67.6493; -41.7142; 35.5*pi/180]; 

% noise.R = diag([0.05 0.05 0.001]); % (x, y, th)  [0.05 0.05 0.001]
noise.R = diag([0.05 0.05 0.001]); % (x, y, th)  [0.05 0.05 0.001]
noise.Q = diag([5000 3000 800]);   % (range, angle, signature)     [1 0.01]

% variables
t = 2000; % overall time, maximum 61945

alpha = 0.03;%0.03; % Minimum PI value
u = zeros(2, t - 1); % control signal 2 * (t-1) [speed; steering angle]
z = []; % measurement 5 * unknown
        % [distance; direction; radius of tree; time index; correspondence]
x = zeros(3, t); % estimated state 3 * t [x; y; phi]
m = []; % landmarks 3 * unknown [x; y; r]
mObs = []; % time moments that each landmark is observed 
omega = []; % information matrix
xi = []; % information vector
sigma = 1e-12*eye(3);

tic

% initialize (from the graph slam implementation, just to read control signals)
[u, z, x0, m, mObs] = initialize(controlSpeed, controlSteering, controlTime, ...
                                 laserData, laserTime, t, u, z, x0, m, mObs);

mu = x0(:, 1);
x = zeros(3, t);
x(:, 1) = x0(:, 1);

%% Initialize figure.
figure('name','SLAM Demo','color','w','units','normalized',...
         'outerposition',[0 0 1 1]);
hold on;box on; axis equal;
image([-142 188],[260 -120],I,'alphadata',1);  % Satelite map
axis([-120 160 -50 240]); 
plot(Lo_m(1:7:end,1)+67.6493,La_m(1:7:end,1)+41.7142, 'b.', 'markersize', 5);  % Ground truth
hold on;
set(gca,'xtick',[],'ytick',[]); 
set(gca,'Color','w','XColor','w','YColor','w')
odoPath     = plot(0,0,'r-','linewidth',1.5,'erasemode','normal');
obsFeature  = plot(0,0,'y+','linewidth',1.5,'markersize',8,'erasemode','normal');

%% Just for legend
truth_       = plot(0,0,'b.','linewidth',3,'markersize',8,'erasemode','normal');
odoPath_     = plot(0,0,'r-','linewidth',1.5,'erasemode','normal');
obsFeature_  = plot(0,0,'y+','linewidth',1.5,'markersize',8,'erasemode','normal');
label        = [odoPath_,truth_, obsFeature_];
lgd          = legend(label,'Estimated Path','GPS','Features');
set(lgd,'box','on','position',[0.61 0.83 0.1 0.1],'color','w',...
    'FontSize',11,'fontname','song','fontweight','bold');


for i = 1: t - 1
    [mu_bar, sigma_bar] = prediction(mu, sigma, u(:, i));
%     mu_bar(3)
    zt = z(:, find(z(4,:)==i));
    % Data association
    if size(zt, 2) > 0
        for noi = 1:size(zt, 2)
            flag = 0; % Whether to append a landmark
            %disp(zt);
            
            tree_x = mu_bar(1) + zt(1, noi) * cos(zt(2, noi) + mu_bar(3) - pi/2);
            tree_y = mu_bar(2) + zt(1, noi) * sin(zt(2, noi) + mu_bar(3) - pi/2);
            tree_r = zt(3, noi);
            H = cell(N_t + 1);
            PSI = cell(N_t + 1);
            zhat = cell(N_t + 1);
            if N_t > 0
                mu_xy = mu_bar;
%                 mu_xy = zeros(3 * N_t + 3, 1);
%                 for lmi = 1:N_t
%                     mu_xy(3 * lmi + 1) = mu_bar(1) + mu_bar(3 * lmi + 1) * cos(mu_bar(3) + mu_bar(3 * lmi + 2));
%                     mu_xy(3 * lmi + 2) = mu_bar(2) + mu_bar(3 * lmi + 1) * sin(mu_bar(3) + mu_bar(3 * lmi + 2));
%                     mu_xy(3 * lmi + 3) = mu_bar(3 * lmi + 3); 
%                 end
%                 PI = zeros(N_t + 1, 1);
                PI = zeros(N_t + 1, 1);
                for k = 1:N_t
                    delta = [mu_xy(3 * k + 1) - mu_bar(1);
                        mu_xy(3 * k + 2) - mu_bar(2)];
                    q = delta' * delta;
                    zhat{k} = [sqrt(q);
                        atan2(delta(2), delta(1)) - mu_bar(3);
                        mu_xy(3 * k + 3)];
                    F = [eye(3) zeros(3, 3 * N_t);
                        zeros(3, 3 * k) eye(3) zeros(3, 3 * (N_t - k))];
                    H_matrix = [-sqrt(q) * delta(1) -sqrt(q) * delta(2) 0 sqrt(q) * delta(1) sqrt(q) * delta(2) 0;
                                delta(2) -delta(1) -1 -delta(2) delta(1) 0;
                                0 0 0 0 0 1];
                    H{k} = 1/q * H_matrix * F;
                    PSI{k} = H{k} * sigma_bar * H{k}' + noise.Q;
                    PI(k) = (zt(1:3, noi) - zhat{k})' * inv(PSI{k}) * (zt(1:3, noi) - zhat{k});
                end
                PI(N_t + 1) = alpha;
%                 if N_t > 1
%                     [~, j2] = mink(PI, 2);
%                     j = j2(1);
%                     if PI(j) > alpha || PI(j2(2)) < 1.1 * PI(j)
%                         flag = 1;
%                     end
%                 else
                [minP, j] = min(PI);
                if j == N_t + 1
                    flag = 1;
                end
%                 end
%                 if PI(j) > alpha
%                     flag = 1;
%                 end                
            else
                flag = 1;
                j = N_t + 1;
            end
            if flag == 1
                % Increase the dimension
                mu_bar = [mu_bar; tree_x; tree_y; tree_r];
                sigma_bar = blkdiag(sigma_bar, 1e3 * eye(3));

                delta = [tree_x - mu_bar(1); tree_y - mu_bar(2)];
                q = delta' * delta;
                zhat{N_t + 1} = [sqrt(q); atan2(delta(2), delta(1)); tree_r];
                F = [eye(3) zeros(3, 3 * (N_t + 1)); zeros(3, 3 * (N_t + 1)) eye(3)];
                H_matrix = [-sqrt(q) * delta(1) -sqrt(q) * delta(2) 0 sqrt(q) * delta(1) sqrt(q) * delta(2) 0;
                            delta(2) -delta(1) -1 -delta(2) delta(1) 0;
                            0 0 0 0 0 1];
                H{N_t + 1} = 1/q * H_matrix * F;
                PSI{N_t + 1} = H{N_t + 1} * sigma_bar * H{N_t + 1}' + noise.Q;
                PI(N_t + 1) = (zt(1:3, noi) - zhat{N_t + 1})' * inv(PSI{N_t + 1}) * (zt(1:3, noi) - zhat{N_t + 1});

                N_t = N_t + 1;
                size(mu_bar, 1);
            end
            % Update
            K = sigma_bar * H{j}' * inv(PSI{j});
    %             nor = norm((zt(1:3, noi) - zhat{j}))
%             temp = [mu_bar(1); mu_bar(2)];
            mu_bar = mu_bar + K * (zt(1:3, noi) - zhat{j});
%             [mu_bar(1); mu_bar(2)] - temp

            sigma_bar = sigma_bar - K * H{j} * sigma_bar;
        end
    end
%     mu_bar(1:3)
%     x0(3, i + 1)
    mu_bar(3) = mod(mu_bar(3), 2 * pi);
    mu = mu_bar;
    x(:, i + 1) = mu(1:3);
    sigma = sigma_bar;


    set(odoPath,  'xdata', x(1, 1:i) - x0(1, 1), ...
                 'ydata', x(2, 1:i) - x0(2, 1));
    % Observations
    set(obsFeature, 'xdata', mu_xy(4:3:end) - x0(1, 1),...
        'ydata', mu_xy(5:3:end) - x0(2, 1));
    drawnow;
    hold on;
end

time = toc;

% plot error
gps_t = 180;
pose_errors = zeros(2, gps_t);
mu_ptr = 1;
for i = 1:gps_t
    if (controlTime(mu_ptr) < timeGps(i))
        mu_ptr = mu_ptr + 1;
        if (mu_ptr > t) 
            gps_t = i - 1;
            break;
        end
    end
    pose_errors(1, i) = x(1, mu_ptr) - Lo_m(i);
    pose_errors(2, i) = x(2, mu_ptr) - La_m(i);
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
