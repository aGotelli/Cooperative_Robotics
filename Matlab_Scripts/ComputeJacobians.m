function [uvms] = ComputeJacobians(uvms)
% compute the relevant Jacobians here
% joint limits
% manipulability
% tool-frame position control
% vehicle-frame position control
% horizontal attitude 
% minimum altitude
% preferred arm posture ( [-0.0031 1.2586 0.0128 -1.2460] )
%
% remember: the control vector is:
% [q_dot; p_dot] 
% [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
% with the vehicle velocities projected on <v>
%
% therefore all task jacobians should be of dimensions
% m x 13
% where m is the row dimension of the task, and of its reference rate

% computation for tool-frame Jacobian
% [omegax_t omegay_t omegaz_t xdot_t ydot_t zdot_t] = Jt ydot
% [angular velocities; linear velocities]
%
% Ste is the rigid body transformation from vehicle-frame to end-effector
% frame projected on <v>
uvms.Ste = [eye(3) zeros(3);  -skew(uvms.vTe(1:3,1:3)*uvms.eTt(1:3,4)) eye(3)];

% uvms.bJe contains the arm end-effector Jacobian (6x7) wrt arm base
% top three rows are angular velocities, bottom three linear velocities
uvms.Jt_a  = uvms.Ste * [uvms.vTb(1:3,1:3) zeros(3,3); zeros(3,3) uvms.vTb(1:3,1:3)] * uvms.bJe;

% vehicle contribution is simply a rigid body transformation from vehicle
% frame to tool frame. Notice that linear and angular velocities are
% swapped due to the different definitions of the task and control
% variables
uvms.Jt_v = [zeros(3) eye(3); eye(3) -skew(uvms.vTt(1:3,4))];

% juxtapose the two Jacobians to obtain the global one
uvms.Jt = [uvms.Jt_a uvms.Jt_v];

%% Exercise 1
% Vehicle position/attitude control jacobians
uvms.Jvpos = [zeros(3, 7), uvms.wTv(1:3, 1:3), zeros(3, 3)];

uvms.Jvatt = [zeros(3, 7), zeros(3, 3),  uvms.wTv(1:3, 1:3)];

%% Vehicle horizontal attitude
w_kw = [0, 0, 1]';
v_kv = [0, 0, 1]';
v_kw = uvms.vTw(1:3, 1:3) * w_kw;

% rho is the misalignment vector, norm(rho) is the angle
uvms.v_rho = ReducedVersorLemma(v_kw, v_kv);

if norm(uvms.v_rho) ~= 0
    
    v_n = uvms.v_rho / norm(uvms.v_rho);
    
else

    v_n = [0, 0, 0]';
   
end
    
uvms.Jha = [zeros(1, 7), zeros(1, 3), v_n'];

%% Exercise 2
% Vehicle altitude control jacobian
v_d = uvms.sensorDistance * v_kv;

% Find the actual altitude of the vehicle
uvms.alt = (v_d' * v_kw);

% The actual altitude of the vehicle, assuming a flat seafloor, depends
% only on z_dot
uvms.Jalt = [zeros(1, 7), v_kw', zeros(1, 3)];

%% Constrained task (underactuated)
uvms.Jua = [zeros(6, 7), eye(6)];

%% Exercise 4
% Compute the projection on the horizontal plane of the distance vector 
% from the goal to the current vehicle position 
w_d = uvms.wTg_v(1:3, 4) - uvms.wTv(1:3, 4);
w_d_proj = w_d - ((w_d' * w_kw) * w_kw);

% Get the corresponding versor
if norm(w_d_proj) ~= 0
    
    w_d_proj_vers = w_d_proj / norm(w_d_proj);
    
else

    w_d_proj_vers = [0, 0, 0]';
   
end

v_d = uvms.vTw(1:3, 1:3) * w_d;
v_d_proj_vers = uvms.vTw(1:3, 1:3) * w_d_proj_vers;
v_iv = [1, 0, 0]';

% Compute the misalignment vector and its versor
misalignVector = ReducedVersorLemma(v_iv, v_d_proj_vers);

if norm(misalignVector) ~= 0
    
    rho_vers = misalignVector / norm(misalignVector);
    
else

    rho_vers = [0, 0, 0]';
   
end

% Store the misalignment angle theta for the task reference
uvms.theta = norm(misalignVector);

% Obtain the jacobian matrix
uvms.JhorAlign = rho_vers' * [zeros(3, 7), -(1 / (norm(v_d) * norm(v_d))) * skew(v_d), -eye(3)];

end