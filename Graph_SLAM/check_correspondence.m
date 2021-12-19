function [z, mNew, taoNew] = check_correspondence(z, x, m, tao, omega, cov)
    
    % check correspondence for non-corresponding features
    landmarkNum = size(m, 2);
    grouping = zeros(1, landmarkNum); 
    % 0 means only detected once, k means it is the same landmark as other landmark with grouping value k
    groupingNum = 0;
    % number of groups of corresponding features
    landmarkNumNew = landmarkNum;
    for j = 1 : landmarkNum - 1
        for k = (j + 1) : landmarkNum
            probability = correspondence_test(x, m, tao, omega, cov, j, k);
            if (probability > 1)
                %z(z(5, :) == k) = j;
                if (grouping(j) == 0)
                    groupingNum = groupingNum + 1;
                    grouping(j) = groupingNum;
                end
                grouping(k) = grouping(j);
                landmarkNumNew = landmarkNumNew - 1;
            end
        end
    end
    
    % update map
    mapping = zeros(1, landmarkNum); 
    % mapping(j) indicates which new landmark corresponds to the old landmark k
    mNew = zeros(3, landmarkNumNew);
    mPtr = 1;
    for i = 1 : landmarkNum
        if (grouping(i) == 0)
            mNew(:, mPtr) = m(:, i);
            mapping(i) = mPtr;
        elseif (grouping(i) ~= -1)
            mNew(:, mPtr) = [mean(m(1, grouping == grouping(i))); ...
                mean(m(2, grouping == grouping(i))); mean(m(3, grouping == grouping(i)))];
            mapping(grouping == grouping(i)) = mPtr;
            grouping(grouping == grouping(i)) = -1;
        end
        mPtr = mPtr + 1;
    end
    
    % update tao
    taoNew = cell(1, landmarkNumNew);
    for i = 1 : landmarkNumNew
        taoNew{i} = union(tao{mapping == i});
    end
    
    % update correspondence
    for i = 1 : size(z, 2)
        z(5, i) = mapping(z(5, i));
    end
end