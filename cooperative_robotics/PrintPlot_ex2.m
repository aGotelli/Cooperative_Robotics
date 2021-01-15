function [ ] = PrintPlot_ex2( plt )
%% Plot the 3D path and the misalignment vector
figure
hold on
rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates

% Plot the initial position, the goal position and the robot's path
scatter3(plt.initPos(1), plt.initPos(2), plt.initPos(3), 50, 'b', 'filled');
scatter3(plt.goalPos(1), plt.goalPos(2), plt.goalPos(3), 50, 'g', 'filled');
plot3(plt.history.x, plt.history.y, plt.history.z, 'r', 'LineWidth', 2);
% Comment if not needed
scatter3(rock_center(1),rock_center(2),rock_center(3), 50, 'k', 'filled');

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
legend('Initial position', 'Goal position', '3D path', 'Rock center' , 'Misalignment vector')
% legend('Initial position', 'Goal position', '3D path', 'Misalignment vector')
% legend('Initial position', '3D path')
title("Robot's path")
end