function [ ] = PrintPlot_ex1( plt )

%% Plot the robot position and attitude
figure(1);
s1 = subplot(2,1,1);
hplot = plot(plt.t, plt.p(1:3, :));
set(hplot, 'LineWidth', 1);
xlabel("Time [s]")
ylabel("Position [m]")
legend('x','y','z')
title(s1, "Robot position")
s2 = subplot(2,1,2);
hplot = plot(plt.t, plt.p(4:6, :));
set(hplot, 'LineWidth', 1);
xlabel("Time [s]")
ylabel("Attitude [rad]")
legend('roll','pitch','yaw');
title(s2, "Robot attitude")

%% Plot the robot linear and angular velocities
figure(2);
s1 = subplot(2,1,1);
hplot = plot(plt.t, plt.p_dot(1:3, :));
set(hplot, 'LineWidth', 1);
xlabel("Time [s]")
ylabel("Linear velocity [m/s]")
legend('xdot', 'ydot','zdot')
title(s1, "Robot linear velocities")
s2 = subplot(2,1,2);
hplot = plot(plt.t, plt.p_dot(4:6, :));
set(hplot, 'LineWidth', 1);
xlabel("Time [s]")
ylabel("Angular velocity [rad/s]")
legend('omega_x','omega_y','omega_z');
title(s2, "Robot angular velocities")

%% Plot the 3D path and the misalignment vector
figure(3)
hold on
% rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates

% Plot the initial position, the goal position and the robot's path
scatter3(plt.initPos(1), plt.initPos(2), plt.initPos(3), 50, 'b', 'filled');
scatter3(plt.goalPos(1), plt.goalPos(2), plt.goalPos(3), 50, 'g', 'filled');
plot3(plt.history.x, plt.history.y, plt.history.z, 'r', 'LineWidth', 2);
% Comment if not needed
% scatter3(rock_center(1),rock_center(2),rock_center(3), 50, 'k', 'filled');

% Plot the misalignment vector every 2 seconds
for index = 1:size(plt.axis, 2)
    
    hold on
    
    plot3([plt.vehi.x(index); plt.vehi.x(index) + plt.ang.x(index)], ...
          [plt.vehi.y(index); plt.vehi.y(index) + plt.ang.y(index)], ...
          [plt.vehi.z(index); plt.vehi.z(index) + plt.ang.z(index)], 'k');
    view(45, 45);
end
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
% legend('Initial position', 'Goal position', '3D path', 'Rock center' , 'Misalignment vector')
legend('Initial position', 'Goal position', '3D path', 'Misalignment vector')
title("Robot's path")

%% Plot the Horizontal Attitude activation function
figure(4);
hplot = plot(plt.t, plt.a(9,:));
set(hplot, 'LineWidth', 2);
xlabel("Time [s]")
legend('Aha');
title("Horizontal attitude activation function")


%% Plot the Horizontal Attitude activation function
figure(5);
hplot = plot((1:size(plt.angTool, 2)) * 2, plt.angTool);
set(hplot, 'LineWidth', 2);
xlabel("Time [s]")
title("Norm of the end-effector's misalignment vector")

%% Plot the 2D path with the sea floor and the thresholds
% figure(6)
% % initially distance = 0, ground is wrong
% ground = plt.history.z - plt.distance;
% % ground = ground(100:end);
% thresh_hard = ground + plt.min_offset;
% thresh_soft = ground + plt.min_offset + plt.range;
% 
% hold on
% plot(plt.history.x,plt.history.z,'LineWidth',2)
% plot(plt.history.x,ground, 'r')
% plot(plt.history.x,thresh_hard, 'k')
% plot(plt.history.x,thresh_soft, 'g-')
% legend('Path', 'Sea floor', 'Hard threshold', 'Soft threshold')
% title('Vehicle 2D path with sea floor and thresholds')
% hold off

end

