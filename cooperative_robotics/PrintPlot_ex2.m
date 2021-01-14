function [ ] = PrintPlot_ex2( plt )

% some predefined plots
% you can add your own

% figure(1);
% subplot(2,1,1);
% hplot = plot(plt.t, plt.q);
% set(hplot, 'LineWidth', 1);
% legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
% subplot(2,1,2);
% hplot = plot(plt.t, plt.q_dot);
% set(hplot, 'LineWidth', 1);
% legend('qdot_1','qdot_2','qdot_3','qdot_4','qdot_5','qdot_6','qdot_7');
% 
% 
% figure(2);
% subplot(3,1,1);
% hplot = plot(plt.t, plt.p);
% set(hplot, 'LineWidth', 1);
% legend('x','y','z','roll','pitch','yaw');
% subplot(2,1,2);
% hplot = plot(plt.t, plt.p_dot);
% set(hplot, 'LineWidth', 1);
% legend('xdot', 'ydot','zdot','omega_x','omega_y','omega_z');


% figure(3);
% hplot = plot(plt.t, plt.a(1:7,:));
% set(hplot, 'LineWidth', 2);
% legend('Ajl_11','Ajl_22','Ajl_33','Ajl_44','Ajl_55','Ajl_66','Ajl_77');
% 
% figure(4);
% hplot = plot(plt.t, plt.a(8:9,:));
% set(hplot, 'LineWidth', 2);
% legend('Amu', 'Aha');


%% Plot the 3D path and the misalignment vector
% figure(5)
% hold on
% rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates
% 
% % Plot the initial position, the goal position and the robot's path
% scatter3(plt.initPos(1), plt.initPos(2), plt.initPos(3), 50, 'b', 'filled');
% scatter3(plt.goalPos(1), plt.goalPos(2), plt.goalPos(3), 50, 'g', 'filled');
% plot3(plt.history.x, plt.history.y, plt.history.z, 'r', 'LineWidth', 2);
% % Comment if not needed
% scatter3(rock_center(1),rock_center(2),rock_center(3), 50, 'k', 'filled');
% % Plot the misalignment vector every 2 seconds
% for index = 1:size(plt.axis, 2)
%     
%     hold on
%     
%     plot3([plt.vehi.x(index); plt.vehi.x(index) + plt.misal_rock.x(index)], ...
%           [plt.vehi.y(index); plt.vehi.y(index) + plt.misal_rock.y(index)], ...
%           [plt.vehi.z(index); plt.vehi.z(index) + plt.misal_rock.z(index)], 'k', 'LineWidth', 2);
%     view(45, 45);
% end
% xlabel('x [m]');
% ylabel('y [m]');
% zlabel('z [m]');
% legend('Initial position', 'Goal position', '3D path', 'Rock center' , 'Misalignment vector')
% title("Robot's path")
% 
% scale = 0.2;

%%  Plot the frames
% figure(6)
% hold on
% % Comment if not needed
% p4 = scatter3(rock_center(1),rock_center(2),rock_center(3), 50, 'k', 'filled');
% 
% % Plot the initial vehicle frame
% p1 = plot3([plt.initPos(1); plt.initPos(1) + scale*plt.wRv(1, 1)], ...
%       [plt.initPos(2); plt.initPos(2) + scale*plt.wRv(2, 1)], ...
%       [plt.initPos(3); plt.initPos(3) + scale*plt.wRv(3, 1)], 'b');
%     
% plot3([plt.initPos(1); plt.initPos(1) + scale*plt.wRv(1, 2)], ...
%       [plt.initPos(2); plt.initPos(2) + scale*plt.wRv(2, 2)], ...
%       [plt.initPos(3); plt.initPos(3) + scale*plt.wRv(3, 2)], 'b');
%          
% plot3([plt.initPos(1); plt.initPos(1) + scale*plt.wRv(1, 3)], ...
%       [plt.initPos(2); plt.initPos(2) + scale*plt.wRv(2, 3)], ...
%       [plt.initPos(3); plt.initPos(3) + scale*plt.wRv(3, 3)], 'b');
%   
% % Plot the goal frame
% p2 = plot3([plt.goalPos(1); plt.goalPos(1) + scale*plt.wRg(1, 1)], ...
%       [plt.goalPos(2); plt.goalPos(2) + scale*plt.wRg(2, 1)], ...
%       [plt.goalPos(3); plt.goalPos(3) + scale*plt.wRg(3, 1)], 'g');
%     
% plot3([plt.goalPos(1); plt.goalPos(1) + scale*plt.wRg(1, 2)], ...
%       [plt.goalPos(2); plt.goalPos(2) + scale*plt.wRg(2, 2)], ...
%       [plt.goalPos(3); plt.goalPos(3) + scale*plt.wRg(3, 2)], 'g');
%          
% plot3([plt.goalPos(1); plt.goalPos(1) + scale*plt.wRg(1, 3)], ...
%       [plt.goalPos(2); plt.goalPos(2) + scale*plt.wRg(2, 3)], ...
%       [plt.goalPos(3); plt.goalPos(3) + scale*plt.wRg(3, 3)], 'g');
%   
% % Plot the vehicle frame every 2 seconds
% for index = 1:size(plt.axis, 2)
%     
%     hold on
%     
%     p3 = plot3([plt.vehi.x(index); plt.vehi.x(index) + scale*plt.axis(index).x(1)], ...
%           [plt.vehi.y(index); plt.vehi.y(index) + scale*plt.axis(index).x(2)], ...
%           [plt.vehi.z(index); plt.vehi.z(index) + scale*plt.axis(index).x(3)], 'r');
%     
%     plot3([plt.vehi.x(index); plt.vehi.x(index) + scale*plt.axis(index).y(1)], ...
%           [plt.vehi.y(index); plt.vehi.y(index) + scale*plt.axis(index).y(2)], ...
%           [plt.vehi.z(index); plt.vehi.z(index) + scale*plt.axis(index).y(3)], 'r');
%          
%     plot3([plt.vehi.x(index); plt.vehi.x(index) + scale*plt.axis(index).z(1)], ...
%           [plt.vehi.y(index); plt.vehi.y(index) + scale*plt.axis(index).z(2)], ...
%           [plt.vehi.z(index); plt.vehi.z(index) + scale*plt.axis(index).z(3)], 'r');
%     
% end
% view(45, 45);
% xlabel('x [m]');
% ylabel('y [m]');
% zlabel('z [m]');
% legend([p1, p2, p3, p4], 'Initial vehicle frame', 'Goal frame', 'Vehicle frames', 'Rock center')
% title('Frames')
% 
% hold off
% 
% %% Plot the 2D path with the sea floor and the thresholds
% figure(7)
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

