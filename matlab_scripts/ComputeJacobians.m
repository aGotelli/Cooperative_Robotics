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

% == ORIGINAL (TOOL FRAME) ==
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


% == VEHICLE FRAME == 
uvms.Jvpos = [zeros(3,7)  uvms.wTv(1:3,1:3) zeros(3)]; 
uvms.Jvatt = [zeros(3,7)  zeros(3)          uvms.wTv(1:3,1:3)];

% == HORIZONTAL ATTITUDE ==
w_kw = [0 0 1]';
v_kv = [0 0 1]';
v_kw = uvms.vTw(1:3,1:3) * w_kw;

% misalignment between v_kw and v_kw
uvms.v_rho = ReducedVersorLemma(v_kw,v_kv);
v_n = uvms.v_rho/norm(uvms.v_rho);

% Horizontal attitude Jacobian
uvms.Jha = [zeros(1,7) zeros(1,3) v_n'];

% == VEHICLE ALTITUDE ==
% sensorDistance vector projected on v 
v_sensor = [0 0 uvms.sensorDistance]';
% sensorDistance vector projected on w
w_sensor = uvms.wTv(1:3,1:3) * v_sensor;
% projection of W_sensor on u direction
uvms.w_d = w_kw' * w_sensor;

% Vehicle altitude Jacobian
uvms.Jva = [zeros(1,7) w_kw'*uvms.wTv(1:3,1:3) zeros(1,3)];

% == Alignment of x axis with target direction ==
v_iv = [1,0,0]';
w_iv = uvms.wTv(1:3,1:3) * v_iv;

w_Ov = uvms.wTv(1:3,4);
w_Orock = [12.2025   37.3748  -39.8860]';
w_d_rock = w_Orock - w_Ov;

% projection of distance between veihcle and rock on xy world plane
w_dproj = w_d_rock - w_kw .* w_d_rock;

% unit vector of the projection
w_n_dproj = w_dproj/norm(w_dproj);
% misalignment between x-axis of the vehicle and the unit vector
uvms.v_rho_rock = ReducedVersorLemma(w_iv, w_n_dproj);
w_n_rock = uvms.v_rho_rock/norm(uvms.v_rho_rock);

% Vehicle rock attitude Jacobian
uvms.Jra = w_n_rock' * [zeros(3,7) -(1/(norm(w_dproj'))^2)*skew(w_dproj)*uvms.wTv(1:3,1:3) -eye(3,3)*uvms.wTv(1:3,1:3)]; 

end







