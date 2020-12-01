
addpath('./simulation_scripts');
clc;
clear;
close all

%   So for the plot here you shold plot the derivative of the vector, you
%   will observe a constant vector which norm decreses. So we need to plot
%   this vector plus we need to adjust the axis lenght 

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 40;
loop = 1;
maxloops = ceil(end_time/deltat);

% Rotation matrix to convert coordinates between Unity and the <w> frame
% do not change
wuRw = rotation(0,-pi/2,pi/2);
vRvu = rotation(-pi/2,0,-pi/2);

% pipe parameters
u_pipe_center = [-10.66 31.47 -1.94]'; % in unity coordinates
pipe_center = wuRw'*u_pipe_center;     % in world frame coordinates
pipe_radius = 0.3;

% rock position 
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

% initialize uvms structure
uvms = InitUVMS('Robust');

%% Define actions and initialize the mission structure
% Tool position = 1
% Horizontal attitude = 2
% Vehicle position = 3
% Vehicle attitude = 4
% Vehicle minimum altitude = 5
% Vehicle altitude = 6

% this struct can be used to evolve what the UVMS has to do
mission.phase = 1;
mission.phase_time = 0;
mission.switch = 0;
mission.activationFunctions = {uvms.A.t, uvms.A.ha, uvms.A.v_pos, uvms.A.v_att, uvms.A.minAlt, uvms.A.landing};
mission.totalNumOfTasks = numel(mission.activationFunctions);

% uvms.q 
uvms.q = [-0.0031 0 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]'; 

%% Point 1.1
% % Initial position
% uvms.p = [10.5 35.5 -36 0 0 pi/2]';
% 
% % Defines the goal position for the vehicle position and attitude task
% uvms.goalPosition_v = [10.5   37.5  -38]';
% uvms.wRg_v = rotation(0, 0, 0);
% 
% % Actions definition
% mission.actionNavigation = [2, 3, 4];
% mission.actionToolNavigation = [1, 2, 3, 4];
% 
% mission.currentAction = mission.actionToolNavigation;
% 

%% Point 1.2
% Initial position
uvms.p = [48.5 11.5 -33 0 0 -pi/2]'; 

% defines the goal position for the vehicle position and attitude task
uvms.goalPosition_v = [50   -12.5  -33]';
uvms.wRg_v = rotation(0, 0, -pi/2);

% Actions definition
mission.actionSafeNavigation = [2, 3, 4, 5];

mission.currentAction = mission.actionSafeNavigation;

%% Initialization
uvms.initPosition = uvms.p(1:3)';
uvms.initRotation = rotation(uvms.p(4), uvms.p(5), uvms.p(6));
uvms.wTg_v = [uvms.wRg_v uvms.goalPosition_v; 0 0 0 1];

mission.currentAction = mission.actionSafeNavigation;
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
    
    % receive altitude information from unity
    uvms = ReceiveUdpPackets(uvms, uAltitude);
  
    ydotbar = zeros(13,1);
    Qp = eye(13); 
    
    %   SAFETY MINIMUM ALTITUDE TASK
    [Qp, ydotbar] = iCAT_task(uvms.A.minAlt,    uvms.JminAlt,    Qp, ydotbar, uvms.xdot.minAlt,  0.0001,   0.01, 10);
    
    %   HORIZONTAL ATTITUDE TASK
    [Qp, ydotbar] = iCAT_task(uvms.A.ha,    uvms.Jha,    Qp, ydotbar, uvms.xdot.ha,  0.0001,   0.01, 10);
    
    %   POSITION TASK
    [Qp, ydotbar] = iCAT_task(uvms.A.v_pos,    uvms.Jv_pos,    Qp, ydotbar, uvms.xdot.v_pos,  0.0001,   0.01, 10);
   
    %   ATTITUDE TASK
    [Qp, ydotbar] = iCAT_task(uvms.A.v_att,    uvms.Jv_att,    Qp, ydotbar, uvms.xdot.v_att,  0.0001,   0.01, 10);
    
    %   TOOL FRAME TASK
    %[Qp, ydotbar] = iCAT_task(uvms.A.t,    uvms.Jt,    Qp, ydotbar, uvms.xdot.t,  0.0001,   0.01, 10);
    
    [Qp, ydotbar] = iCAT_task(eye(13),     eye(13),    Qp, ydotbar, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    
    
    % get the two variables for integration
    uvms.q_dot = ydotbar(1:7);
    uvms.p_dot = ydotbar(8:13);
    
    % Integration
	uvms.q = uvms.q + uvms.q_dot*deltat;
    % beware: p_dot should be projected on <v>
    uvms.p = integrate_vehicle(uvms.p, uvms.p_dot, deltat);
    
    % check if the mission phase should be changed
    [uvms, mission] = UpdateMissionPhase_ex1(uvms, mission);
    
    % send packets to Unity viewer
    SendUdpPackets(uvms,wuRw,vRvu,uArm,uVehicle);
        
    % collect data for plots
    plt = UpdateDataPlot(plt,uvms,t,loop);
    loop = loop + 1;
   
    % add debug prints here
    if (mod(t,0.1) == 0)
        t
        altitude = uvms.w_a
        [ang, lin] = CartError(uvms.wTg_v , uvms.wTv);
        distance = norm(lin)
    end

    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

PrintPlot(plt);

