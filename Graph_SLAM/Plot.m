function  Plot(x,m,t_gps)


%car pose graph
plot(x(1,:),x(2,:),'red');
hold on
%landmark graph
plot(m(1,:),m(2,:),'green.','MarkerSize', 8);
hold on
%GPS graph
load aa3_gpsx ;
Lo_m_1=Lo_m(1:t_gps,:) + 67.6493;
La_m_1=La_m(1:t_gps,:) + 41.7142;
plot(Lo_m_1,La_m_1,'.') ;

end