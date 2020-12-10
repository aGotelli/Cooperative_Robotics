function [uvms] = InitUVMS(robotname)

% uvms.vTb
% transformation matrix betwene the arm base wrt vehicle frame
% expresses how the base of the arm is attached to the vehicle
% do NOT change, since it must be coherent with the visualization tool
if (strcmp(robotname, 'DexROV'))    
    % do NOT change
    uvms.vTb = [rotation(pi, 0, pi) [0.167 0 -0.43]'; 0 0 0 1]; 
else
    if (strcmp(robotname, 'Robust'))
        % do NOT change
        uvms.vTb = [rotation(0, 0, pi) [0.85 0 -0.42]'; 0 0 0 1];
    end
end

uvms.q_dot = [0 0 0 0 0 0 0]';
uvms.p_dot = [0 0 0 0 0 0]';

% joint limits corresponding to the actual MARIS arm configuration
uvms.jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
uvms.jlmax  = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];

% to be computed at each time step
uvms.wTv = eye(4,4);
uvms.wTt = eye(4,4);
uvms.wTg = eye(4,4);
uvms.vTw = eye(4,4);
uvms.vTe = eye(4,4);
uvms.vTt = eye(4,4);
uvms.vTg = eye(4,4);
uvms.Ste = eye(6,6);
uvms.bTe = eye(4,4);
uvms.bJe = eye(6,7);
uvms.djdq = zeros(6,7,7);
uvms.mu  = 0;
uvms.phi = zeros(3,1);
uvms.sensorDistance = 0;

uvms.wTg_v = eye(4,4);
uvms.wRg_v = eye(3,3);
uvms.goalPosition_v = zeros(3,1);

uvms.Jjl = [];
uvms.Jmu = [];
uvms.Jt_a = [];
uvms.Jt_v = [];
uvms.Jt = [];

uvms.xdot.jl = [];
uvms.xdot.mu = [];
uvms.xdot.t = [];
    
uvms.A.jl = zeros(7,7);
uvms.A.mu = 0;
uvms.A.t = zeros(6,6);

%%   INITIALIZATION FOR HORIZONTAL ATTITUDE CONTROL
uvms.A.ha = zeros(1,1);
uvms.Jha = [];
uvms.xdot.ha = [];
uvms.v_rho = [];

%%   INITIALIZATION FOR ATTITUDE AND POSITION CONTROL
uvms.A.v_pos = zeros(3, 3);
uvms.A.v_att = zeros(3, 3);
uvms.Jatt = [];
uvms.xdot.v_pos = [];
uvms.xdot.v_att = [];
uvms.ang = [];

%%   ADDING THE DEFINITION FOR THE TASK ENSURING MINIMUM ALTITUDE
uvms.JminAlt = [];
uvms.xdot.minAlt = 0;
uvms.A.minAlt = 0;

uvms.min_offset = 1;
uvms.range_offset = 0.5;

uvms.w_a = 0;
uvms.w_a_previous = 0;

%%   ADDING THE DEFINITION FOR THE LANDING TASK 
uvms.Jlanding = [];
uvms.xdot.landing = 0;
uvms.A.landing = 0;

%%   ADDING THE DEFINITION FOR THE HORIZONTAL ALIGNMENT TO TARGET
uvms.JhorAlign = [];
uvms.xdot.horAlign = 0;
uvms.A.horAlign = 0;

uvms.theta = 0;

%%   ADDING THE DEFINITION FOR ENSURING THE GOAL IN THE ARM WORKSPACE
uvms.dist_to_goal_proj = 0;
uvms.xd = 0;
uvms.yd = 0;
uvms.JdistGoal = [];
uvms.A.distGoal = 0;
uvms.xdot.distGoal = 0;
uvms.ensured_distance_x = 1.5;
uvms.ensured_distance_y = 0.5;

%%  ADDING THE DEFINITION FOR CONSTRAINING VELOCITIES
uvms.Jconstraint = [];
uvms.xdot.constraint = zeros(6,1);
uvms.A.constraint = zeros(6);

%%  ADDING THE DEFINITION FOR AVOIDING JOINT LIMITS
uvms.JjointLimits = [];
uvms.xdot.jointLimits = zeros(7, 1);
uvms.A.lbJointLimits = zeros(7);
uvms.A.ubJointLimits = zeros(7);

end

