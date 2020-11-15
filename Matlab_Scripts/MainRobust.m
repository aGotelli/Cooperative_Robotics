function MainRobust
addpath('./simulation_scripts');
clc;
clear;
close all

%% Initialization
% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 25;
loop = 1;
maxloops = ceil(end_time/deltat);

% Rotation matrix to convert coordinates between Unity and the <w> frame
% do not change
wuRw = rotation(0,-pi/2,pi/2);
vRvu = rotation(-pi/2,0,-pi/2);

% Pipe parameters
u_pipe_center = [-10.66 31.47 -1.94]'; % in unity coordinates
pipe_center = wuRw'*u_pipe_center;     % in world frame coordinates
pipe_radius = 0.3;

% Rock position 
rock_center = [12.2025   37.3748  -39.8860]'; % in world frame coordinates

% UDP Connection with Unity viewer v2
uArm = udp('127.0.0.1',15000,'OutputDatagramPacketSize',28);
uVehicle = udp('127.0.0.1',15001,'OutputDatagramPacketSize',24);
fopen(uVehicle);
fopen(uArm);
uAltitude = dsp.UDPReceiver('LocalIPPort',15003,'MessageDataType','single');
uAltitude.setup();

% Preallocation
plt = InitDataPlot(maxloops);

% Initialize uvms structure
uvms = InitUVMS('Robust');

%% Define actions and initialize the mission structure
% Tool position = 1
% Horizontal attitude = 2
% Vehicle position = 3
% Vehicle attitude = 4
% Vehicle minimum altitude = 5
% Vehicle altitude = 6
% Underactuation = 7
% Horizontal alignment to the goal frame = 8

% This struct can be used to evolve what the UVMS has to do
mission.phase = 1;
mission.phase_time = 0;
mission.switch = 0;
mission.activationFunctions = {uvms.A.t, uvms.A.ha, uvms.A.vpos, uvms.A.vatt, uvms.A.minAlt, uvms.A.alt, uvms.A.ua, uvms.A.horAlign};
mission.totalNumOfTasks = numel(mission.activationFunctions);

%% Initial configuration
% uvms.q 
% Initial joint positions. You can change these values to initialize the 
% simulation with a different starting position for the arm
uvms.q = [-0.0031 0 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]'; 

% uvms.p
% Initial position of the vehicle
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)

% defines the tool control point
uvms.eTt = eye(4);

%% Tool Position
% Init position for the e-e position task
% uvms.p = [8.5 38.5 -38   0 -0.06 0.5]'; 

% defines the goal position for the end-effector/tool position task
% uvms.goalPosition_t = [12.2025   37.3748  -39.8860]';
% uvms.wRg_t = rotation(0, pi, pi/2);
% uvms.wTg_t = [uvms.wRg_t uvms.goalPosition_t; 0 0 0 1];

%% Exercise 1
% Init position for the vehicle position/attitude control task
% uvms.p = [10.5, 35.5, -36, 0, 0, pi/2]';
% 
% % Defines the goal position for the vehicle position/attitude control task
% uvms.goalPosition_v = [10.5, 37.5, -38]';
% uvms.wRg_v = rotation(0, 0, 0);
% uvms.wTg_v = [uvms.wRg_v, uvms.goalPosition_v; 0, 0, 0, 1];

%% Exercise 2
% Init position for the vehicle altitude control task
% uvms.p = [10.5, 35.5, -36, 0, 0, pi/2]';

% CASE 1
% Defines the goal position for the vehicle altitude control task
% uvms.goalPosition_v = [12.2025, 37.3748, -39.886]';
% uvms.wRg_v = rotation(0, 0, 0);
% uvms.wTg_v = [uvms.wRg_v, uvms.goalPosition_v; 0, 0, 0, 1];

% CASE 2
% Defines the goal position for the vehicle altitude control task
% uvms.goalPosition_v = [10.5, 37.5, -41]';
% uvms.wRg_v = rotation(0, 0, 0);
% uvms.wTg_v = [uvms.wRg_v, uvms.goalPosition_v; 0, 0, 0, 1];

%% Exercise 3 (mission)
% Init position for the two actions implementation
% uvms.p = [48, -11.5, -33, 0, 0, 0]';

% Defines the goal position for the two actions implementation
% uvms.goalPosition_v = [50, -12.5, -33]';
% uvms.wRg_v = rotation(0, 0, 0);
% uvms.wTg_v = [uvms.wRg_v, uvms.goalPosition_v; 0, 0, 0, 1];

% mission.actionSafeNavigation = [2, 3, 4, 5];
mission.actionLanding = [2, 3, 6];

% mission.currentAction = mission.actionSafeNavigation;
% mission.previousAction = [];

%% Moving the vehicle and then the tool frame
% Init position for the vehicle position/attitude control task
% uvms.p = [10.5, 35.5, -36, 0, 0, pi/2]';

% Defines the goal position of the end-effector/tool frame
% uvms.goalPosition_t = [12.2025   37.3748  -39.8860]';
% uvms.wRg_t = rotation(0, pi, pi/2);
% uvms.wTg_t = [uvms.wRg_t uvms.goalPosition_t; 0 0 0 1];

% Defines the goal position of the vehicle frame
% uvms.goalPosition_v = [10.5, 37.5, -38]';
% uvms.wRg_v = rotation(0, 0, 0);
% uvms.wTg_v = [uvms.wRg_v, uvms.goalPosition_v; 0, 0, 0, 1];
% 
mission.vehicleControl = [2, 3, 4];
mission.toolControl = [1, 2];
% 
% mission.currentAction = mission.vehicleControl;
% mission.previousAction = [];

%% Constrained task (underactuated)
% Init position for the underactuated task
% uvms.p = [10.5, 35.5, -36, 0, 0, pi/2]';

% Defines the goal position of the end-effector/tool frame
% uvms.goalPosition_t = [12.2025   37.3748  -39.8860]';
% uvms.wRg_t = rotation(0, pi, pi/2);
% uvms.wTg_t = [uvms.wRg_t uvms.goalPosition_t; 0 0 0 1];

% Defines the goal position of the vehicle frame
% uvms.goalPosition_v = [10.5, 37.5, -38]';
% uvms.wRg_v = rotation(0, 0, 0);
% uvms.wTg_v = [uvms.wRg_v, uvms.goalPosition_v; 0, 0, 0, 1];

mission.vehicleControlUnderactuated = [2, 3, 4, 7];
mission.toolControlUnderactuated = [1, 2, 3, 4, 7];

% mission.currentAction = mission.vehicleControlUnderactuated;
% mission.previousAction = [];

%% Exercise 4
% Init position for the improved landing task
uvms.p = [6.5, 31.5, -36, 0, 0, 0]';

% Defines the goal position of the end-effector/tool frame
uvms.goalPosition_t = [12.2025   37.3748  -39.8860]';
uvms.wRg_t = rotation(0, pi, pi/2);
uvms.wTg_t = [uvms.wRg_t uvms.goalPosition_t; 0 0 0 1];

% Defines the goal position of the vehicle frame
uvms.goalPosition_v = [12.2025   37.3748  -37.8860]';
uvms.wRg_v = rotation(pi/3, 0, 0);
uvms.wTg_v = [uvms.wRg_v, uvms.goalPosition_v; 0, 0, 0, 1];

mission.actionSafeNavigationAligned = [2, 3, 4, 5, 8];
mission.alignedLanding = [2, 3, 6, 8];

mission.currentAction = mission.actionSafeNavigationAligned;
mission.previousAction = [];

tic
for t = 0:deltat:end_time
    
    % update all the involved variables
    uvms = UpdateTransforms(uvms);
    uvms = ComputeJacobians(uvms);
    uvms = ComputeTaskReferences(uvms, mission);
    uvms = ComputeActivationFunctions(uvms, mission);
    
    % receive altitude information from unity
    uvms = ReceiveUdpPackets(uvms, uAltitude);
    
    % main kinematic algorithm initialization
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <v>
    ydotbar = zeros(13,1);
    Qp = eye(13);
    
    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
    
    %% Tool Position ---
%     [Qp, ydotbar] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, ydotbar, uvms.xdot.t,  0.0001,   0.01, 10);

    %% Exercise 2
%     [Qp, ydotbar] = iCAT_task(uvms.A.alt, uvms.Jalt, Qp, ydotbar, uvms.xdot.alt, 0.0001, 0.01, 10);

    %% Vehicle Hor. Att.
%     [Qp, ydotbar] = iCAT_task(uvms.A.ha, uvms.Jha, Qp, ydotbar, uvms.xdot.ha, 0.0001, 0.01, 10);

    %% Exercise 1
    % Vehicle position/attitude control task definition
%     [Qp, ydotbar] = iCAT_task(uvms.A.vpos, uvms.Jvpos, Qp, ydotbar, uvms.xdot.vpos, 0.0001, 0.01, 10);
%     [Qp, ydotbar] = iCAT_task(uvms.A.vatt, uvms.Jvatt, Qp, ydotbar, uvms.xdot.vatt, 0.0001, 0.01, 10);
    
    %% Exercise 3 (mission)
%     [Qp, ydotbar] = iCAT_task(uvms.A.minAlt, uvms.Jalt, Qp, ydotbar, uvms.xdot.minAlt, 0.0001, 0.01, 10);
%     [Qp, ydotbar] = iCAT_task(uvms.A.alt, uvms.Jalt, Qp, ydotbar, uvms.xdot.alt, 0.0001, 0.01, 10);
%     [Qp, ydotbar] = iCAT_task(uvms.A.ha, uvms.Jha, Qp, ydotbar, uvms.xdot.ha, 0.0001, 0.01, 10);
%     [Qp, ydotbar] = iCAT_task(uvms.A.vpos, uvms.Jvpos, Qp, ydotbar, uvms.xdot.vpos, 0.0001, 0.01, 10);
%     [Qp, ydotbar] = iCAT_task(uvms.A.vatt, uvms.Jvatt, Qp, ydotbar, uvms.xdot.vatt, 0.0001, 0.01, 10);
    
    %% Moving the vehicle and then the tool frame
%     [Qp, ydotbar] = iCAT_task(uvms.A.ha, uvms.Jha, Qp, ydotbar, uvms.xdot.ha, 0.0001, 0.01, 10);
%     [Qp, ydotbar] = iCAT_task(uvms.A.vpos, uvms.Jvpos, Qp, ydotbar, uvms.xdot.vpos, 0.0001, 0.01, 10);
%     [Qp, ydotbar] = iCAT_task(uvms.A.vatt, uvms.Jvatt, Qp, ydotbar, uvms.xdot.vatt, 0.0001, 0.01, 10);
%     [Qp, ydotbar] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, ydotbar, uvms.xdot.t,  0.0001,   0.01, 10);
    
    %% Constrained task (underactuated)
%     [Qp, ydotbar] = iCAT_task(uvms.A.ua, uvms.Jua, Qp, ydotbar, uvms.xdot.ua, 0.0001, 0.01, 10);
%     [Qp, ydotbar] = iCAT_task(uvms.A.ha, uvms.Jha, Qp, ydotbar, uvms.xdot.ha, 0.0001, 0.01, 10);
%     [Qp, ydotbar] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, ydotbar, uvms.xdot.t,  0.0001,   0.01, 10);
%     [Qp, ydotbar] = iCAT_task(uvms.A.vpos, uvms.Jvpos, Qp, ydotbar, uvms.xdot.vpos, 0.0001, 0.01, 10);
%     [Qp, ydotbar] = iCAT_task(uvms.A.vatt, uvms.Jvatt, Qp, ydotbar, uvms.xdot.vatt, 0.0001, 0.01, 10);

    %% Exercise 4
    [Qp, ydotbar] = iCAT_task(uvms.A.minAlt, uvms.Jalt, Qp, ydotbar, uvms.xdot.minAlt, 0.0001, 0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.horAlign, uvms.JhorAlign, Qp, ydotbar, uvms.xdot.horAlign, 0.0001, 0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.alt, uvms.Jalt, Qp, ydotbar, uvms.xdot.alt, 0.0001, 0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.ha, uvms.Jha, Qp, ydotbar, uvms.xdot.ha, 0.0001, 0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.vpos, uvms.Jvpos, Qp, ydotbar, uvms.xdot.vpos, 0.0001, 0.01, 10);
    [Qp, ydotbar] = iCAT_task(uvms.A.vatt, uvms.Jvatt, Qp, ydotbar, uvms.xdot.vatt, 0.0001, 0.01, 10);
    
    % this task should be the last one
    [Qp, ydotbar] = iCAT_task(eye(13), eye(13), Qp, ydotbar, zeros(13,1), 0.0001, 0.01, 10); 
    
    % get the two variables for integration
    uvms.q_dot = ydotbar(1:7);
    uvms.p_dot = ydotbar(8:13);
    
    % Integration
	uvms.q = uvms.q + uvms.q_dot*deltat;
    
    % beware: p_dot should be projected on <v>
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);
    
    % check if the mission phase should be changed
    mission.phase_time = mission.phase_time + deltat;
    [uvms, mission] = UpdateMissionPhase(uvms, mission);
    
    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
        
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop);
    loop = loop + 1;
   
    % add debug prints here
    if (mod(t,0.1) == 0)
        t
        theta = uvms.theta
        phase = mission.phase
    end

    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

PrintPlot(plt);

end