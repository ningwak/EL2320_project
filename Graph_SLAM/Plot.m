function  Plot(x,m,t)
figure(1)
clear all
%car pose graph
plot(x(1,:),x(2,:),'red');
hold on
%landmark graph
plot(m(1,:),m(2,:),'blue.','MarkerSize', 5);
hold on
%GPS graph
load aa3_gpsx ;
t=200;
Lo_m_1=Lo_m(1:t,:);
La_m_1=La_m(1:t,:);
plot(Lo_m_1,La_m_1,'.') ;

end