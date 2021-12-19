function [car,map,cov] = solve(omega_hat,xi_hat,omega,xi,mObs,x,m)

cov=inv(omega_hat);
car=cov*xi_hat;
map=zeros(3*size(m,2),1);
tau=mObs;

%for each feature do
for i=1:size(m,2)
    index_j = findInformation(x, i, "map")%should complete
    index_tauj=findInformation(x, tau{i}, "car")%(3*(tau{i}-1)+1):3*tau{i};
    
    map((3*i-2):(3*i)) = inv(omega(index_j, index_j)) * (xi(index_j) + omega(index_j, index_tauj) * car(index_tauj));
end


fprintf("solve successfully\n");

end