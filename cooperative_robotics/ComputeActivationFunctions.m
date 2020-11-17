function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

% arm tool position control
% always active
uvms.A.t = eye(6);


%   ATTITUDE AND POSITION CONTROL
uvms.A.v_pos = eye(3);
uvms.A.v_att = eye(3);

%   ACTIVATION FUNCTION FOR THE TASK ENSURING OFFSET 
v_d = [0 0 uvms.sensorDistance]';
w_d = uvms.wTv(1:3, 1:3)*v_d;
k = [0 0 1]';
uvms.w_a = k' * w_d;
uvms.A.z_offset = DecreasingBellShapedFunction(uvms.min_offset, (uvms.min_offset + uvms.range_offset), 0, 1, uvms.w_a);

%   ACTIVATION FUNCTION FOR THE FOR LANDING
v_d = [0 0 uvms.sensorDistance]';
w_d = uvms.wTv(1:3, 1:3)*v_d;
k = [0 0 1]';
uvms.w_a = k' * w_d;
uvms.A.landing = 1;






