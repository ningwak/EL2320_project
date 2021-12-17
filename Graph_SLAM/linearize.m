function [omega,xi] = linearize(z,x0,u,m,t)

global noise
R = noise.R;
Q = noise.Q;

dt=0.025;
a=3.78;
b=0.5;
H=0.76;
L=2.83;

%initialize information matrix omega and information vector xi
dim_num = t+size(m,2);%t:number of motion states,m:number of landmarks
omega=zeros(3*dim_num, 3*dim_num);
xi=zeros(3*dim_num, 1);

omega_start=eye(3)*(10^10); %set an infinite start information matrix
omega(1:3, 1:3)=omega_start;

for i=1:t-1 %for all controls
    
    v = u(1, i);
    steer = u(2, i); %steering angle
    phi = x0(3, i);  % heading angle
    %calculate jacobians
    G=[1 0 -dt*v*(sin(phi) + 1/L*tan(steer)*(a*cos(phi) - b*sin(phi))); 
         0 1  dt*v*(cos(phi) - 1/L*tan(steer)*(a*sin(phi) + b*cos(phi)));
         0 0  1];
    
    % update the information matrix omega and information vector xi
    %index1 = (3*(i-1)+1):3*i;
    %index2 = (3*i+1):3*(i+1);
    index=(3*(i-1)+1):3*(i+1);
    omega(index,index)=omega(index,index)+[inv(R),- R \ G;(- R \ G)',G' / R * G]; %??? maybe should adeverse

end

for j=1:size(z,2)
    jj=obj.z.c(j);; % false
    t=obj.z.idx(j); % false

    delta=[m(1,jj)-x0(1,t);m(2,jj)-x0(2,t)];
    q=delta'*delta;

    z_hat = [sqrt(q);atan2(delta(2),delta(1))-x0(3,t)+pi/2;m(3,jj)];

    index_1 = (3*(t-1)+1):3*t;
    index_2 = (3*(jj-1)+1+3*size(x0,2)):(3*jj+3*size(x0,2));
    
    %calculate H
    H_1 = 1/q * [ -sqrt(q) * delta(1), -sqrt(q) * delta(2), 0; delta(2), -delta(1), -q; 0,0,0];
    H_2 = 1/q * [ sqrt(q) * delta(1), sqrt(q) * delta(2), 0; -delta(2), delta(1), 0; 0,0,q];
    
    %add to information matrix H
    omega(index_1,index_1) = omega(index_1,index_1) + H_1' /Q * H_1;
    omega(index_1,index_2) = omega(index_1,index_2) + H_1' /Q * H_2;
    omega(index_2,index_1) = omega(index_2,index_1) + H_2' /Q * H_1;
    omega(index_2,index_2) = omega(index_2,index_2) + H_2' /Q * H_2;

    %add to information vector xi
    xi_1= H_1' / Q * (m(:,j) - z_hat + [H_1, H_2]*[x0(:,t); m(:,jj)]); %here is a little different with hu
    xi_2= H_2' / Q * (m(:,j) - z_hat + [H_1, H_2]*[x0(:,t); m(:,jj)]); %here is a little different with hu
    xi(t_idx) = xi(index_1) + xi_1;
    xi(j_idx) = xi(index_2) + xi_2;
end

end