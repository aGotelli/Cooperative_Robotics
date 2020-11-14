function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

% reference for vehicle frame position control task
[w_vang, w_vlin] = CartError(uvms.wTgV , uvms.wTv);
% limit the requested velocities...

uvms.xdot.vpos(1:3,:) = Saturate(0.4*w_vlin, 0.2);
uvms.xdot.vatt(1:3,:) = Saturate(0.4*w_vang, 0.2);

% horizontal attitude task reference
uvms.xdot.vha = 0.4 * (0-norm(uvms.v_rho));

% vehicle altitude task reference

uvms.xdot.vva = Saturate(0.4 * (2 - uvms.w_d), 0.2);

% vehicle altitude for landing
uvms.xdot.vvland = Saturate(0.4 * (0.1 - uvms.w_d), 0.4);

% vehicle attitude towards rock
uvms.xdot.vra = 0.4 * (0 - norm(uvms.v_rho_rock));
end
    



