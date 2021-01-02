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

% Null initialization to avoid uninitialized errors;
uvms.goalPosition_v = [0 0 0]';
uvms.wRg_v = rotation(0, 0, 0);

%% Define actions and initialize the mission structure
% Tool position = 1
% Horizontal attitude = 2
% Vehicle position = 3
% Vehicle attitude = 4
% Vehicle minimum altitude = 5  Not usable
% Vehicle altitude control = 6  Not usable
% Horizontal alignment to target = 7
% Ensuring distance from the tool target = 8
% Constrain velocities = 9
% Avoid lower bound joint limits = 10
% Avoid upper bound joint limits = 11
% Ensure preferred position for the arm = 12
% Arm vehicle coordination = 13

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
                               uvms.A.armPrefPos,...
                               uvms.A.armVehiCoord};
mission.totalNumOfTasks = numel(mission.activationFunctions);

% initial arm position
uvms.q = [-0.0044    1.1582    0.0175   -1.0710    3.0318    0.0876    1.1906]';

%%  EXERCISE 6

% Initial position
%uvms.p = [-1.9379 10.4813-6.1 -29.7242-0.1 0 0 0]';
uvms.p = [-1.8446   10.8430  -29.5    0.0065   -0.0001    1.0838]';


% Initial goal position definition
% slightly over the top of the pipe
distanceGoalWrtPipe = 0.1;
uvms.goalPosition = pipe_center + (pipe_radius + distanceGoalWrtPipe)*[0 0 1]';
uvms.wRg = rotation(pi,0,0);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

% Actions definition
mission.actionTool = [1, 2, 7, 12, 13];

mission.currentAction = mission.actionTool;

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
    [Qp, rhop] = iCAT_task(uvms.A.armVehiCoord,    uvms.JarmVehiCoord,    Qp, rhop, uvms.xdot.armVehiCoord,  0.0001,   0.01, 10);
    
    %   CONSTRAIN VELOCITIES
    %[Qp, rhop] = iCAT_task(uvms.A.constraint,    uvms.Jconstraint,    Qp, rhop, uvms.xdot.constraint,  0.0001,   0.01, 10);
    
    %   AVOID LOWER BOUND JOINT LIMITS
    [Qp, rhop] = iCAT_task(uvms.A.lbJointLimits,    uvms.JjointLimits,    Qp, rhop, uvms.xdot.jointLimits,  0.0001,   0.01, 10);
    
    %   AVOID UPPER BOUND JOINT LIMITS
    [Qp, rhop] = iCAT_task(uvms.A.ubJointLimits,    uvms.JjointLimits,    Qp, rhop, uvms.xdot.jointLimits,  0.0001,   0.01, 10);
    
    %   SAFETY MINIMUM ALTITUDE TASK 
    %[Qp, rhop] = iCAT_task(uvms.A.minAlt,    uvms.JminAlt,    Qp, rhop, uvms.xdot.minAlt,  0.0001,   0.01, 10);
    
    %   HORIZONTAL ATTITUDE TASK 
    [Qp, rhop] = iCAT_task(uvms.A.ha,    uvms.Jha,    Qp, rhop, uvms.xdot.ha,  0.0001,   0.01, 10);
    
    %   HORIZONTAL ALIGNMENT TO TARGET TASK
    [Qp, rhop] = iCAT_task(uvms.A.horAlign,    uvms.JhorAlign,    Qp, rhop, uvms.xdot.horAlign,  0.0001,   0.01, 10);
    
    %   LANDING TASK
    %[Qp, rhop] = iCAT_task(uvms.A.landing,    uvms.Jlanding,    Qp, rhop, uvms.xdot.landing,  0.0001,   0.01, 10);
    
    %   POSITION TASK
    [Qp, rhop] = iCAT_task(uvms.A.v_pos,    uvms.Jv_pos,    Qp, rhop, uvms.xdot.v_pos,  0.0001,   0.01, 10);
   
    %   ATTITUDE TASK
    [Qp, rhop] = iCAT_task(uvms.A.v_att,    uvms.Jv_att,    Qp, rhop, uvms.xdot.v_att,  0.0001,   0.01, 10);
    
    %   DISTANCE FROM THE TOOL TARGET
    [Qp, rhop] = iCAT_task(uvms.A.distGoal,    uvms.JdistGoal,    Qp, rhop, uvms.xdot.distGoal,  0.0001,   0.01, 10);
    
    %   TOOL FRAME TASK
    [Qp, rhop] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, rhop, uvms.xdot.t,  0.0001,   0.01, 10);
    
    %   OPTIMAL ARM POSITION FRAME TASK
    [Qp, rhop] = iCAT_task(uvms.A.armPrefPos,    uvms.JarmPrefPos,    Qp, rhop, uvms.xdot.armPrefPos,  0.0001,   0.01, 10);
    
    [Qp, rhop] = iCAT_task(eye(13),     eye(13),    Qp, rhop, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    % get the two variables for integration
    uvms.q_dot = rhop(1:7);
    uvms.p_dot = rhop(8:13);
    
     % Integration
	uvms.q = uvms.q + uvms.q_dot*deltat;
    
    %   Exercise 6 add sinusiodal disturbance to the vehicle positions
    disturb = [0 0.025 0]';
    disturb_ang = [0 0 0]';
    disturb_ang = disturb_ang*0.5*sin(0.5*t*2*pi);
    uvms.p_dot(1:3) = uvms.wTv(1:3,1:3)*disturb;
    uvms.p_dot(4:6) = uvms.wTv(1:3,1:3)*disturb_ang;
    
    % beware: p_dot should be projected on <v>
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);

    mission.phase_time = mission.phase_time + deltat;
   
    
    % check if the mission phase should be changed
    [uvms, mission] = UpdateMissionPhase_ex6(uvms, mission);
    
    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
        
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop);
    loop = loop + 1;
   
    % add debug prints here
    if (mod(t,0.1) == 0)
        t
        phase = mission.phase
        %uvms.q'
    end
    
    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

PrintPlot_ex6(plt);