function  Plot(x,m,t_gps)

figure;
%car pose graph
plot(x(1,:),x(2,:),'red');
hold on
%landmark graph
% angle = linspace(0, 2 * pi, 100);
% for i = 1:size(m, 2)
%     xx = m(1, i) + m(3, i) .* cos(angle);
%     yy = m(2, i) + m(3, i) .* sin(angle);
%     plot(xx, yy, 'blue');
%     hold on;
% end
plot(m(1,:),m(2,:),'green.','Marker', '.','MarkerSize', 5);
hold on
%GPS graph
load aa3_gpsx.mat;
Lo_m_1=Lo_m(1:t_gps,:) + 67.6493;
La_m_1=La_m(1:t_gps,:) + 41.7142;
plot(Lo_m_1,La_m_1,'.') ;

end