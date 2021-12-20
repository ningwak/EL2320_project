function [pose,map,cov] = solve(omega_hat,xi_hat,omega,xi,tau,x,m)

cov=inv(omega_hat);
pose=cov*xi_hat;
map=zeros(3*size(m,2),1);

%for each feature do
for i=1:size(m,2)
    index_j = find_information_index(x, i, "map");
    index_tauj=find_information_index(x, tau{i}, "car");
    
    map((3*i-2):(3*i)) = (omega(index_j, index_j)) \ (xi(index_j) + omega(index_j, index_tauj) * pose(index_tauj));
   
end

pose = reshape(pose, 3, []);
map = reshape(map, 3, []);

fprintf("solve successfully\n");

end