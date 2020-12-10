addpath('./simulation_scripts');
clc;
clear;
close all

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 15;
loop = 1;
maxloops = ceil(end_time/deltat);

% this struct can be used to evolve what the UVMS has to do
mission.phase = 1;
mission.phase_time = 0;

% Rotation matrix to convert coordinates between Unity and the <w> frame
% do not change
wuRw = rotation(0,-pi/2,pi/2);
vRvu = rotation(-pi/2,0,-pi/2);

% pipe parameters
u_pipe_center = [-10.66 31.47 -1.94]'; % in unity coordinates
pipe_center = wuRw'*u_pipe_center;     % in world frame coordinates
pipe_radius = 0.3;

% UDP Connection with Unity viewer v2
uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
fopen(uVehicle);
fopen(uArm);

% Preallocation
plt = InitDataPlot(maxloops);

% initialize uvms structure
uvms = InitUVMS('DexROV');

%% Define actions and initialize the mission structure
% Tool position = 1
% Horizontal attitude = 2
% Vehicle position = 3
% Vehicle attitude = 4
% Vehicle minimum altitude = 5
% Vehicle altitude control = 6
% Horizontal alignment to target = 7
% Ensuring distance from the tool target = 8
% Constrain velocities = 9
% Avoid lower bound joint limits = 10
% Avoid upper bound joint limits = 11
% Ensure preferred position for the arm = 12

% this struct can be used to evolve what the UVMS has to do
mission.phase = 1;
mission.phase_time = 0;
mission.switch = 0;
mission.activationFunctions = {uvms.A.t,...
                               uvms.A.ha,...
                               uvms.A.v_pos,...
                               uvms.A.v_att,...
                               uvms.A.minAlt,...
                               uvms.A.landing,...
                               uvms.A.horAlign,...
                               uvms.A.distGoal,...
                               uvms.A.constraint,...
                               uvms.A.lbJointLimits,...
                               uvms.A.ubJointLimits,...
                               uvms.A.armPrefPos};
mission.totalNumOfTasks = numel(mission.activationFunctions);


%%  INITIALIZATION
% uvms.q 
uvms.q = [-0.0031 1.2586 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]';

%%  EXERCISE 5.1
distanceGoalWrtPipe = 0.3;
% Initial position
%uvms.p = [-1.9379 10.4813-6.1 -29.7242-0.1 0 0 0]';
uvms.p = pipe_center + (pipe_radius + 2)*[0 0 1]';
% Defines the goal position for the vehicle position and attitude task
uvms.goalPosition_v = pipe_center + (pipe_radius + distanceGoalWrtPipe)*[0 0 1]';

uvms.wRg_v = rotation(0, 0, 0);

% initial goal position definition
% slightly over the top of the pipe
uvms.goalPosition = pipe_center + (pipe_radius + distanceGoalWrtPipe)*[0 0 1]';
uvms.wRg = rotation(pi,0,0);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

% Actions definition
mission.actionSafeNavigation = [2, 3, 4, 5];
mission.actionFloatingManip = [1, 12];



mission.currentAction = mission.actionSafeNavigation;

%% Initialization
uvms.initPosition = uvms.p(1:3)';
uvms.initRotation = rotation(uvms.p(4), uvms.p(5), uvms.p(6));

uvms.wTg_v = [uvms.wRg_v uvms.goalPosition_v; 0 0 0 1];
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

mission.previousAction = [];

% defines the tool control point
uvms.eTt = eye(4);
tic
for t = 0:deltat:end_time
    % update all the involved variables
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms);
    uvms = ComputeTaskReferences(uvms, mission);
    uvms = ComputeActivationFunctions(uvms, mission);
   
    % main kinematic algorithm initialization
    % rhop order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    rhop = zeros(13,1);
    Qp = eye(13); 
    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
    
    %   CONSTRAIN VELOCITIES
    [Qp, ydotbar] = iCAT_task(uvms.A.constraint,    uvms.Jconstraint,    Qp, ydotbar, uvms.xdot.constraint,  0.0001,   0.01, 10);
    
    %   AVOID LOWER BOUND JOINT LIMITS
    [Qp, ydotbar] = iCAT_task(uvms.A.lbJointLimits,    uvms.JjointLimits,    Qp, ydotbar, uvms.xdot.jointLimits,  0.0001,   0.01, 10);
    
    %   AVOID UPPER BOUND JOINT LIMITS
    [Qp, ydotbar] = iCAT_task(uvms.A.ubJointLimits,    uvms.JjointLimits,    Qp, ydotbar, uvms.xdot.jointLimits,  0.0001,   0.01, 10);
    
    %   SAFETY MINIMUM ALTITUDE TASK 
    [Qp, ydotbar] = iCAT_task(uvms.A.minAlt,    uvms.JminAlt,    Qp, ydotbar, uvms.xdot.minAlt,  0.0001,   0.01, 10);
    
    %   HORIZONTAL ATTITUDE TASK 
    [Qp, ydotbar] = iCAT_task(uvms.A.ha,    uvms.Jha,    Qp, ydotbar, uvms.xdot.ha,  0.0001,   0.01, 10);
    
    %   HORIZONTAL ALIGNMENT TO TARGET TASK
    [Qp, ydotbar] = iCAT_task(uvms.A.horAlign,    uvms.JhorAlign,    Qp, ydotbar, uvms.xdot.horAlign,  0.0001,   0.01, 10);
    
    %   LANDING TASK
    [Qp, ydotbar] = iCAT_task(uvms.A.landing,    uvms.Jlanding,    Qp, ydotbar, uvms.xdot.landing,  0.0001,   0.01, 10);
    
    %   POSITION TASK
    [Qp, ydotbar] = iCAT_task(uvms.A.v_pos,    uvms.Jv_pos,    Qp, ydotbar, uvms.xdot.v_pos,  0.0001,   0.01, 10);
   
    %   ATTITUDE TASK
    [Qp, ydotbar] = iCAT_task(uvms.A.v_att,    uvms.Jv_att,    Qp, ydotbar, uvms.xdot.v_att,  0.0001,   0.01, 10);
    
    %   DISTANCE FROM THE TOOL TARGET
    [Qp, ydotbar] = iCAT_task(uvms.A.distGoal,    uvms.JdistGoal,    Qp, ydotbar, uvms.xdot.distGoal,  0.0001,   0.01, 10);
    
    %   TOOL FRAME TASK
    [Qp, ydotbar] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, ydotbar, uvms.xdot.t,  0.0001,   0.01, 10);
    
    
    [Qp, rhop] = iCAT_task(eye(13),     eye(13),    Qp, rhop, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    % get the two variables for integration
    uvms.q_dot = rhop(1:7);
    uvms.p_dot = rhop(8:13);
    
    % Integration
	uvms.q = uvms.q + uvms.q_dot*deltat;
    % beware: p_dot should be projected on <v>
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);
    
    % check if the mission phase should be changed
    [uvms, mission] = UpdateMissionPhase_ex5(uvms, mission);
    
    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
        
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop);
    loop = loop + 1;
   
    % add debug prints here
    if (mod(t,0.1) == 0)
        t
        uvms.p'
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

PrintPlot(plt);
