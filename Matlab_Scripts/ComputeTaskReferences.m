function [uvms] = ComputeTaskReferences(uvms, mission)
% Compute the task references here

% Reference for tool-frame position control task
[ang, lin] = CartError(uvms.wTg_t , uvms.wTt);
uvms.xdot.t = 0.2 * [ang; lin];

% Limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

%% Exercise 1
% Reference for vehicle position/attitude control task
% w_vang and w_vlin are w.r.t the <w> frame
[w_vang, w_vlin] = CartError(uvms.wTg_v, uvms.wTv);
uvms.xdot.vpos = 0.5 * w_vlin;
uvms.xdot.vatt = 0.5 * w_vang;

% Limit the requested velocities...
uvms.xdot.vpos = Saturate(uvms.xdot.vpos, 0.5);
uvms.xdot.vatt = Saturate(uvms.xdot.vatt, 0.5);

%% Vehicle horizontal attitude
% xdot = lambda * (x_reference - x)
% If x_reference = 0 it means that we want to drive the variable to zero
uvms.xdot.ha = 0.2 * (0 - norm(uvms.v_rho));

%% Exercise 2
% Reference for vehicle minimum altitude control task
% We want the vehicle altitude to go to at least 1.5m 
uvms.xdot.minAlt = 0.5 * (1.5 - uvms.alt);

%% Exercise 3
% Reference for vehicle altitude control task
uvms.xdot.alt = 0.2 * (0 - uvms.alt);

%% Constrained task (underactuated)
uvms.xdot.ua = uvms.p_dot;

%% Exercise 4
uvms.xdot.horAlign = 0.2 * (0 - uvms.theta);

end
