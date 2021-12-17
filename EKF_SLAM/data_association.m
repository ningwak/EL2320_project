% This function performs the data association.
% Inputs:
%           mu_bar(t)             (3N + 3) x 1   
%           sigma_bar(t)          (3N + 3) x (3N + 3)
%           z                     Current observation (not associated)
% Outputs:   
%           z                     Current observation (associated)
function z = data_association(mu_bar, sigma_bar, z)

