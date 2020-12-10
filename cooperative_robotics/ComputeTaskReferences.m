function [uvms] = ComputeTaskReferences(uvms, mission)
% compute the task references here

% reference for tool-frame position control task
[ang, lin] = CartError(uvms.vTg , uvms.vTt);
uvms.xdot.t = 0.2 * [ang; lin];
% limit the requested velocities...
uvms.xdot.t(1:3) = Saturate(uvms.xdot.t(1:3), 0.2);
uvms.xdot.t(4:6) = Saturate(uvms.xdot.t(4:6), 0.2);

%%   Vehicle horizontal attitude
% xdot = lambda * (x_reference - x)
% If x_reference = 0 it means that we want to drive the variable to zero
uvms.xdot.ha = 0.2 * (0 - norm(uvms.v_rho));

%%   ATTITUDE AND POSITION CONTROL 
[ang, lin] = CartError(uvms.wTg_v , uvms.wTv);
uvms.ang = ang;

uvms.xdot.v_pos = Saturate(0.5 * lin , 0.5) ;
uvms.xdot.v_att = Saturate(0.5 * ang , 0.5) ;

%%   THE TASK REFERENCE FOR ENSURING THE OFFSET
uvms.xdot.minAlt = Saturate(0.2 * ((uvms.min_offset + uvms.range_offset) - uvms.w_a), 0.2) ;

%%   THE TASK REFERENCE FOR LANDING
uvms.xdot.landing = Saturate(0.5 * ( - uvms.w_a), 0.5) ;

%%   THE TASK REFERENCE FOR THE HORIZONTAL ALIGNMENT TO TARGET
uvms.xdot.horAlign = Saturate(0.5 * (0 - uvms.theta), 0.5);

%%   THE TASK REFERENCE FOR ENSURING THE TARGET IN THE MANIPULATOR WORKSPACE
uvms.xdot.distGoal = [Saturate(0.5 * (uvms.ensured_distance_x - uvms.xd), 0.5) ;
                      Saturate(0.5 * (uvms.ensured_distance_y - uvms.yd), 0.5) ];

%%  THE TASK REFERENCE FOR CONSTRAINING THE VELOCITIES
uvms.xdot.constraint = zeros(6,1);

%%  THE TASK REFERENCE FOR AVOIDING JOINT LIMITS
offset = 0.5;
uvms.xdot.lbjointLimits = uvms.jlmin + offset - uvms.q_dot;
uvms.xdot.ubjointLimits = uvms.jlmax - offset - uvms.q_dot;

%%   THE TASK FOR ENSURING OPTIMAL POSITION FOR THE ARM 
uvms.xdot.armPrefPos = uvms.armPrefPos - uvms.q_dot(1:4);

end

