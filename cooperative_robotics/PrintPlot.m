function [ ] = PrintPlot( plt )

% some predefined plots
% you can add your own

figure(1);
subplot(2,1,1);
hplot = plot(plt.t, plt.q);
set(hplot, 'LineWidth', 1);
legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');


figure(2);
subplot(3,1,1);
hplot = plot(plt.t, plt.p);
set(hplot, 'LineWidth', 1);
legend('x','y','z','roll','pitch','yaw');
subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot);
set(hplot, 'LineWidth', 1);
legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');


figure(3);
hplot = plot(plt.t, plt.a(1:7,:));
set(hplot, 'LineWidth', 2);
legend('Ajl_11','Ajl_22','Ajl_33','Ajl_44','Ajl_55','Ajl_66','Ajl_77');

figure(4);
hplot = plot(plt.t, plt.a(8:9,:));
set(hplot, 'LineWidth', 2);
legend('Amu', 'Aha');



figure(5)
plot3(plt.history.x, plt.history.y, plt.history.z, 'k', 'LineWidth', 2)

hold on

for index = 1:size(plt.axis, 2)
    scale = 0.2;
    
    plot3([plt.vehi.x(index); plt.vehi.x(index) + scale*plt.axis(index).x(1)], ...
            [plt.vehi.y(index); plt.vehi.y(index) + scale*plt.axis(index).x(2)], ...
             [plt.vehi.z(index),; plt.vehi.z(index) + scale*plt.axis(index).x(3)], 'r');
    
    plot3([plt.vehi.x(index); plt.vehi.x(index) + scale*plt.axis(index).y(1)], ...
            [plt.vehi.y(index); plt.vehi.y(index) + scale*plt.axis(index).y(2)], ...
             [plt.vehi.z(index); plt.vehi.z(index) + scale*plt.axis(index).y(3)],'g');
         
    plot3([plt.vehi.x(index); plt.vehi.x(index) + scale*plt.axis(index).z(1)], ...
            [plt.vehi.y(index); plt.vehi.y(index) + scale*plt.axis(index).z(2)], ...
             [plt.vehi.z(index); plt.vehi.z(index) + scale*plt.axis(index).z(3)], 'b');
    
    hold on 
    
end



figure(6)
plot(plt.history.z)
hold on

ground = plt.history.z - plt.distance;
ground = ground(100:end);
thresh_hard = ground + plt.min_offset;
thresh_soft = ground + plt.min_offset + plt.range;
plot(ground, 'r')
plot(thresh_hard, 'k')
plot(thresh_soft, 'g-')
legend('path', 'sea floor', 'hard threshold', 'soft threshold')

    


    

end

