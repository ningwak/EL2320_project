function [omega_hat,xi_hat] = reduce(omega,xi,m,tau,x)%not sure if tao is the set of all poses xt at which j was observed

omega_hat=omega;
xi_hat=xi;

%for each feature do
for i=1:size(m,2)
    index_j = (3*i-2+3*size(x,2)):(3*i+3*size(x,2));
    index_tauj=find_information_index(x, tau{i}, "car");%(3*tau{i}-2):(3*tau{i});

    %subtract```` from xi
    xi_hat(index_tauj) = xi_hat(index_tauj)- omega_hat(index_tauj,index_j)/omega_hat(index_j,index_j)*xi_hat(index_j);

    %subtract```` from omega
    omega_hat(index_tauj,index_tauj) = omega_hat(index_tauj,index_tauj)- omega_hat(index_tauj,index_j)/omega_hat(index_j,index_j)*omega_hat(index_j,index_tauj);
end

%remove from omega and xi all rows/columns corresponding to j
xi_hat=xi_hat(1:3*size(x,2));
omega_hat=omega_hat(1:3*size(x,2),1:3*size(x,2));

fprintf("reduce successfully\n");

end