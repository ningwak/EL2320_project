function [u, z, x, m, tau] = initialize(controlSpeed, controlSteering, controlTime, laserData, laserTime, t, u, z, x, m, tau)
    
    a = 3.78;
    b = 0.5;
    L = 2.83;
    dt = 0.025;
    
    u(1, :) = controlSpeed(1 : t - 1);
    u(2, :) = controlSteering(1 : t - 1);
    
    laserLength = size(laserData, 2);
    laserPtr = 1; % laser data pointer
    landmarkPtr = 0; % landmark pointer
    for i = 1 : (t - 1)
        % predict pose
        v = u(1, i);
        steer = u(2, i);
        phi = x(3, i);
        x(:, i + 1) = x(:, i) + dt .* [v * cos(phi) - v / L * tan(steer) * (a * sin(phi) + b * cos(phi));
                                         v * sin(phi) + v / L * tan(steer) * (a * cos(phi) - b * sin(phi));
                                         v / L * tan(steer)];
        
        if laserPtr <= laserLength
            if laserTime(laserPtr) < controlTime(i)
                % add measurement
                measurement = cell2mat(laserData(laserPtr));
                num = size(measurement, 2);
                zt = [measurement; i * ones(1, num); (landmarkPtr + 1):(landmarkPtr + num)];
                z = [z zt];
                % add landmark
                mt = zeros(3, num);
                mt(1, :) = x(1, i) + zt(1, :) .* cos(zt(2, :) + x(3, i) - pi/2);
                mt(2, :) = x(2, i) + zt(1, :) .* sin(zt(2, :) + x(3, i) - pi/2);
                mt(3, :) = zt(3, :);
                m = [m mt];
                for j = 1:num
                    tau{landmarkPtr + j} = i;
                end
                landmarkPtr = landmarkPtr + num;
                laserPtr = laserPtr + 1;
            end
        end
    end
    
    fprintf("initialize successfully\n");
    
end