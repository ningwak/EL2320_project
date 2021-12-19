function probability = correspondence_test(x, m, tao, omega, cov, j, k, Q)
        
    %fprintf("j%d, k%d", j, k);
    index_jk = find_information_index(x, [j, k], "map");
    index_taojk = find_information_index(x, union(tao{j}, tao{k}), "car");
    omega_jk = omega(index_jk, index_jk) - omega(index_jk, index_taojk) * ...
               cov(index_taojk, index_taojk) * omega(index_taojk, index_jk);
    xi_jk = omega_jk * [m(:, j); m(:, k)];
       
    omega_djk = [eye(3), -eye(3)] * omega_jk * [eye(3); -eye(3)];
    xi_djk = [eye(3), -eye(3)] * xi_jk;
    %mu_djk = omega_djk \ xi_djk;
    mu_djk = [eye(3), -eye(3)] * [m(:, j); m(:, k)];
    %probability = (2 * pi * inv(omega_djk)) ^ (-0.5) * exp(- 0.5 * mu_djk' / omega_djk * mu_djk);
    probability = (2 * pi * det(Q))^(-0.5) * exp(-0.5 * abs(mu_djk' / Q * mu_djk));
              
end