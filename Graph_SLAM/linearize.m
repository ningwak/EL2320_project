function [omega,xi] = linearize(z,x,u,m,t,R,Q)

dt=0.025;
a=3.78;
b=0.5;
H=0.76;
L=2.83;

%initialize information matrix omega and information vector xi
dim_num = t+size(m,2);%t:number of motion states,m:number of landmarks
omega=zeros(3*dim_num, 3*dim_num);
xi=zeros(3*dim_num, 1);

%sparse the matrix
omega=sparse(omega);
xi=sparse(xi);

omega_start=eye(3)*(10^8); %set an infinite start information matrix
omega(1:3, 1:3)=omega_start;

%for all controls
for i=1:t-1 
    
    v = u(1, i);
    steer = u(2, i); %steering angle
    phi = x(3, i);  % heading angle
    %calculate jacobians
    G=[1 0 -dt*v*(sin(phi) + 1/L*tan(steer)*(a*cos(phi) - b*sin(phi))); 
         0 1  dt*v*(cos(phi) - 1/L*tan(steer)*(a*sin(phi) + b*cos(phi)));
         0 0  1];
    
    % update the information matrix omega and information vector xi
    %index1 = (3*(i-1)+1):3*i;
    %index2 = (3*i+1):3*(i+1);
    index=(3*(i-1)+1):3*(i+1);
    omega(index,index)=omega(index,index)+[inv(R),- R \ G;(- R \ G)',G' / R * G]; %??? maybe should adeverse
    
    xi(index)=xi(index)+[inv(R)*(x(:,i+1)-G*x(:,i));-G'*inv(R)*(x(:,i+1)-G*x(:,i))];
    %omega(index,index)=omega(index,index)+[G' / R * G, -G'*inv(R);-inv(R)*G, inv(R)];

end


for j=1:size(z,2)
    jj=z(5,j); % correspondence
    t=z(4,j); % time index

    delta=[m(1,jj)-x(1,t);m(2,jj)-x(2,t)];
    q=delta'*delta;

    z_hat = [sqrt(q);atan2(delta(2),delta(1))-x(3,t)+pi/2;m(3,jj)];

    index_1 = (3*(t-1)+1):3*t;
    index_2 = (3*(jj-1)+1+3*size(x,2)):(3*jj+3*size(x,2));
    
    %calculate H
    H_1 = 1/q * [ -sqrt(q) * delta(1), -sqrt(q) * delta(2), 0; delta(2), -delta(1), -q; 0,0,0];
    H_2 = 1/q * [ sqrt(q) * delta(1), sqrt(q) * delta(2), 0; -delta(2), delta(1), 0; 0,0,q];
    
    %add to information matrix H
    omega(index_1,index_1) = omega(index_1,index_1) + H_1' /Q * H_1;
    omega(index_1,index_2) = omega(index_1,index_2) + H_1' /Q * H_2;
    omega(index_2,index_1) = omega(index_2,index_1) + H_2' /Q * H_1;
    omega(index_2,index_2) = omega(index_2,index_2) + H_2' /Q * H_2;

    %add to information vector xi
    xi_1= H_1' / Q * (z(1:3,j) - z_hat + [H_1, H_2]*[x(:,t); m(:,jj)]); %a little different
    xi_2= H_2' / Q * (z(1:3,j) - z_hat + [H_1, H_2]*[x(:,t); m(:,jj)]); %a little different
    xi(index_1) = xi(index_1) + xi_1;
    xi(index_2) = xi(index_2) + xi_2;
end

fprintf("linearize successfully\n");

end