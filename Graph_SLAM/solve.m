function [x,m,cov] = solve(omega_hat,xi_hat,omega,xi,tao,x,m)

cov=inv(omega_hat);
x=cov*xi_hat;
m=zeros(3*size(m,2),1);
tau=tao;

%for each feature do
for i=1:size(m,2)
    index_j = find_information_index(x, i, "map");
    index_tauj=find_information_index(x, tau{i}, "car");
    
    m((3*i-2):(3*i)) = inv(omega(index_j, index_j)) * (xi(index_j) + omega(index_j, index_tauj) * x(index_tauj));
end

x = reshape(x, 3, []);
m = reshape(m, 3, []);

fprintf("solve successfully\n");

end