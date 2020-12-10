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

%%   HORIZONTAL ATTITUDE
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

%%   ATTITUDE AND POSITION CONTROL 
uvms.Jv_pos = [ zeros(3, 7)     uvms.wTv(1:3, 1:3)      zeros(3, 3)     ];
uvms.Jv_att = [ zeros(3, 7)         zeros(3, 3)     uvms.wTv(1:3, 1:3)  ];

%%   JACOBIAN FOR THE TASK OF ENSURING OFFSET
v_d = [0 0 uvms.sensorDistance]';
w_d = uvms.wTv(1:3, 1:3)*v_d;

k = [0 0 1]';
uvms.w_a = k' * w_d;

uvms.JminAlt = k'*[zeros(3, 7)    uvms.wTv(1:3, 1:3)  zeros(3, 3)];

%%   JACOBIAN FOR THE TASK FOR LANDING
v_d = [0 0 uvms.sensorDistance]';
w_d = uvms.wTv(1:3, 1:3) * v_d;
k = [0 0 1]';
uvms.w_a = k' * w_d;
uvms.Jlanding = k' * [zeros(3, 7)    uvms.wTv(1:3, 1:3)  zeros(3, 3)];

%%   JACOBIAN FOR THE HORIZONTAL ALIGNMENT TO TARGET
% Compute the projection on the horizontal plane of the distance vector 
% from the goal to the current vehicle position 
rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates
w_d = rock_center - uvms.wTv(1:3, 4);
w_d_proj = w_d - ((w_d' * w_kw) * w_kw);

% Get the corresponding versor
if norm(w_d_proj) ~= 0
    
    w_d_proj_vers = w_d_proj / norm(w_d_proj);
    
else

    w_d_proj_vers = [0, 0, 0]';
   
end

v_d_proj = uvms.vTw(1:3, 1:3) * w_d_proj;
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
uvms.JhorAlign = rho_vers' * [zeros(3, 7), -(1 / (norm(v_d_proj) * norm(v_d_proj))) * skew(v_d_proj), -eye(3)];

%%   JACOBIAN FOR ENSURING THE GOAL IN THE ARM WORKSPACE
uvms.dist_to_goal_proj = norm(w_d_proj);
uvms.xd = [1 0 0] * w_d_proj;
uvms.yd = [0 1 0] * w_d_proj;
w_Dxy = [1,0,0; 0,1,0];
         
uvms.JdistGoal = [zeros(2, 7) -w_Dxy * uvms.wTv(1:3, 1:3)  zeros(2, 3)];

%%  JACOBIAN FOR UNDERACTUATION
uvms.Jconstraint = [zeros(6,7) eye(6,6)];

%%  JACOBIAN FOR AVOIDING JOINT LIMITS
uvms.JjointLimits = [eye(7), zeros(7, 6)];

%%  JACOBIAN FOR THE ARM PREFERRED SHAPE 
uvms.JarmPrefPos = [eye(4) zeros(4, 9)];

end