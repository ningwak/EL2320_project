clear;
clc;
load('dataVictoriaPark.mat');

load aa3_gpsx ; 
% plot(Lo_m(1:275),La_m(1:275),'.') ;
plot(Lo_m(1:151),La_m(1:151),'.') ;
hold on;

global vehicle x0 noise N_t INF
vehicle.L = 2.83;
vehicle.a = 0.95;
vehicle.b = 0.5;
vehicle.H = 0.76;
N_t = 0;
x0 = [-67.6493; -41.7142; 35.5*pi/180]; % initial estimated state 3 * t [x; y; phi]
INF = 500;

% noise.R = diag([0.05 0.05 0.001]); % (x, y, th)  [0.05 0.05 0.001]
noise.R = diag([0.05 0.05 0.001]); % (x, y, th)  [0.05 0.05 0.001]
noise.Q = diag([1002 692 1]);   % (range, angle, signature)     [1 0.01]

% variables
t = 1300; % overall time, maximum 61945

alpha = 0.15; % Minimum PI value
u = zeros(2, t - 1); % control signal 2 * (t-1) [speed; steering angle]
z = []; % measurement 5 * unknown
        % [distance; direction; radius of tree; time index; correspondence]
x = zeros(3, t); % estimated state 3 * t [x; y; phi]
m = []; % landmarks 3 * unknown [x; y; r]
mObs = []; % time moments that each landmark is observed 
omega = []; % information matrix
xi = []; % information vector
sigma = 1e-6 * eye(3);

% initialize (from the graph slam implementation, just to read control signals)
[u, z, x0, m, mObs] = initialize(controlSpeed, controlSteering, controlTime, ...
                                 laserData, laserTime, t, u, z, x0, m, mObs);

mu = x0(:, 1);
x = zeros(3, t);
x(:, 1) = x0(:, 1);
for i = 1: t - 1
    [mu_bar, sigma_bar] = prediction(mu, sigma, u(:, i));
%     mu_bar(3)
    zt = z(:, find(z(4,:)==i));
    % Data association
    if size(zt, 2) > 0
        for noi = 1:size(zt, 2)
            flag = 0; % Whether to append a landmark
            tree_x = mu_bar(1) + zt(1, noi) * cos(zt(2, noi) + mu_bar(3));
            tree_y = mu_bar(2) + zt(1, noi) * sin(zt(2, noi) + mu_bar(3));
            tree_r = zt(3, noi);
            H = cell(N_t + 1);
            PSI = cell(N_t + 1);
            zhat = cell(N_t + 1);
            if N_t > 0
                mu_xy = zeros(3 * N_t + 3, 1);
                for lmi = 1:N_t
                    mu_xy(3 * lmi + 1) = mu_bar(1) + mu_bar(3 * lmi + 1) * cos(mu_bar(3) + mu_bar(3 * lmi + 2));
                    mu_xy(3 * lmi + 2) = mu_bar(2) + mu_bar(3 * lmi + 1) * sin(mu_bar(3) + mu_bar(3 * lmi + 2));
                    mu_xy(3 * lmi + 3) = mu_bar(3 * lmi + 3); 
                end
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
                                delta(2) -delta(1) -q -delta(2) delta(1) 0;
                                0 0 0 0 0 q];
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
                [~, j] = min(PI);
                if j == N_t + 1
                    flag = 1;
                end
%                 end
%                 if PI(j) > alpha
%                     flag = 1;
%                 end                
            else
                flag = 1;
            end
            if flag == 1
                % Increase the dimension
                j = N_t + 1;
                mu_bar = [mu_bar; zt(1:3, noi)];
                sigma_bar = blkdiag(sigma_bar, INF * eye(3));

                delta = [tree_x - mu_bar(1); tree_y - mu_bar(2)];
                q = delta' * delta;
                zhat{N_t + 1} = [sqrt(q); atan2(delta(2), delta(1)); tree_r];
                F = [eye(3) zeros(3, 3 * (N_t + 1)); zeros(3, 3 * (N_t + 1)) eye(3)];
                H_matrix = [-sqrt(q) * delta(1) -sqrt(q) * delta(2) 0 sqrt(q) * delta(1) sqrt(q) * delta(2) 0;
                            delta(2) -delta(1) -q -delta(2) delta(1) 0;
                            0 0 0 0 0 q];
                H{N_t + 1} = 1/q * H_matrix * F;
                PSI{N_t + 1} = H{N_t + 1} * sigma_bar * H{N_t + 1}' + noise.Q;
                PI(N_t + 1) = (zt(1:3, noi) - zhat{N_t + 1})' * inv(PSI{N_t + 1}) * (zt(1:3, noi) - zhat{N_t + 1});

                N_t = N_t + 1
            end
            % Update
            K = sigma_bar * H{j}' * inv(PSI{j});
    %             nor = norm((zt(1:3, noi) - zhat{j}))
            mu_bar = mu_bar + K * (zt(1:3, noi) - zhat{j});
            sigma_bar = sigma_bar - K * H{j} * sigma_bar;
        end
    end
%     mu_bar(1:3)
%     x0(3, i + 1)
    i
    mu_bar(3) = mod(mu_bar(3), 2 * pi);
    mu = mu_bar;
    x(:, i + 1) = mu(1:3);
    sigma = sigma_bar;
end
plot(x(1, :), x(2, :), "Color", 'r');
hold on;
for i = 1:N_t - 1
    plot(mu_xy(3 * i + 1), mu_xy(3 * i + 2), 'b.','MarkerSize', 10);
    hold on;
end
plot(x0(1, :), x0(2, :), "Color", 'g');




