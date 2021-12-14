laserData = cell(1, 7249);
for i = 1:7249
    j = 1;
    alphaStartAll = [];
    alphaAll = [];
    thetaAll = [];
    dAll = []; 
    while j < 362
        if (LASER(i, j) < 8000)
            alphaStart = j;
            alphaStartAll = [alphaStartAll alphaStart];
            while j < 361 && LASER(i, j + 1) <8000
                j = j + 1;
            end
            alpha = j - alphaStart + 1;
            alphaAll = [alphaAll alpha];
            theta = alphaStart + alpha / 2;
            thetaAll = [thetaAll theta];
            d = LASER(i, floor(j));
            dAll = [dAll d];
        end
        j = j + 1;
    end
    alphaAll = alphaAll ./2 ./ 180 .* pi;
    thetaAll = (thetaAll - 1)./2 ./ 180 .* pi;
    dAll = double(dAll) ./ 100;
    rAll = sin(alphaAll ./ 2) .* double(dAll) ./ (1 - sin(alphaAll ./ 2));
    distanceAll = dAll + rAll;
    
    [~, rAllindex] = find(rAll > 0.05);
    thetaAll = thetaAll(rAllindex);
    distanceAll = distanceAll(rAllindex);
    rAll = rAll(rAllindex);
    laserData{i} = [thetaAll; distanceAll; rAll];
end
laserTime = TLsr';
controlTime = time';
controlSpeed = speed';
controlSteering = steering';