function probability = correspondence_test(x, tau, omega, cov, j, k, Q, mu)
        
    %fprintf("j%d, k%d", j, k);
    index_jk = find_information_index(x, [j, k], "map");
    index_taujk = find_information_index(x, union(tau{j}, tau{k}), "car");
    omega_jk = omega(index_jk, index_jk) - omega(index_jk, index_taujk) * ...
               cov(index_taujk, index_taujk) * omega(index_taujk, index_jk);
    xi_jk = omega_jk * mu(index_jk);

    %omega_djk = [eye(3), -eye(3)] * omega_jk * [eye(3); -eye(3)];
    %xi_djk = [eye(3), -eye(3)] * xi_jk;
    %mu_djk = omega_djk \ xi_djk;
    %probability = (2 * pi * det(omega_djk)) ^ (-0.5) * exp(- 0.5 * mu_djk' / omega_djk * mu_djk);
    
    mu_djk = [eye(3), -eye(3)] * mu(index_jk);
    probability = (2 * pi * det(Q))^(-0.5) * exp(-0.5 * abs(mu_djk' / Q * mu_djk));

end