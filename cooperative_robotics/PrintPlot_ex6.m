function [ ] = PrintPlot_ex6( plt )

% some predefined plots
% you can add your own

figure(1);
s1 = subplot(3,1,1);
plot(plt.t, plt.t_x, 'r'); 
xlabel(s1, 'Time [s]');
ylabel(s1, 'Position [m]');
title(s1, 'x tool coordinate');
s2 = subplot(3,1,2);
plot(plt.t, plt.t_y, 'g'); 
xlabel(s2, 'Time [s]');
ylabel(s2, 'Position [m]');
title(s2, 'y tool coordinate');
s3 = subplot(3,1,3);
plot(plt.t, plt.t_z, 'b'); 
xlabel(s3, 'Time [s]');
ylabel(s3, 'Position [m]');
title(s3, 'z tool coordinate');

figure(2)
s1 = subplot(2,1,1);
hplot = plot(plt.t, plt.p_dot);
title(s1, 'Vehicle Velocities');
set(hplot, 'LineWidth', 1);
legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');
xlabel(s1, 'Time [s]');
ylabel(s1, 'Velocities');
s2 = subplot(2,1,2);
hplot = plot(plt.t, plt.q_dot);
title(s2, 'Arm Velocities');
set(hplot, 'LineWidth', 1);
legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
xlabel(s2, 'Time [s]');
ylabel(s2, 'Velocities [RAD/s]');

figure(3)
plot(plt.t, plt.xdot_t, 'LineWidth', 1);
title('End Effector Velocities (Inertial)');
legend('omega_x','omega_y','omega_z','xdot', 'ydot','zdot');
xlabel('Time [s]')
ylabel('Velocities')

end

