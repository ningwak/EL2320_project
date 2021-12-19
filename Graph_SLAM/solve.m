function [x,m,cov] = solve(omega_hat,xi_hat,omega,xi,tao,x,m)

cov=inv(omega_hat);
x=cov*xi_hat;
m=zeros(3*size(m,2),1);
tau=tao;

%for each feature do
for i=1:size(m,2)
    index_j = find_information_index(x, i, "map")%should complete
    index_tauj=find_information_index(x, tau{i}, "car")%(3*(tau{i}-1)+1):3*tau{i};
    
    m((3*i-2):(3*i)) = inv(omega(index_j, index_j)) * (xi(index_j) + omega(index_j, index_tauj) * x(index_tauj));
end


fprintf("solve successfully\n");

end