function MainRobust
addpath('./simulation_scripts');
clc;
clear;
close all

% Simulation variables (integration and final time)
deltat = 0.005;
end_time = 35;
loop = 1;
maxloops = ceil(end_time/deltat);

% this struct can be used to evolve what the UVMS has to do
mission.phase = 1;
mission.phase_time = 0;
% mission.actions.A1 = [1,2,3,4,0,0,0];   % move to a point
% mission.actions.A2 = [0,2,3,0,5,6,0];   % landing
% mission.actions.A3 = [0,2,3,0,5,6,7];   % tool task
mission.actions.A1 = [1,2,3,0,5,0,7];   % move to a point
mission.actions.A2 = [0,2,3,0,0,6,7];   % landing
mission.actions.A3 = [1,2,3,4,0,6,7];   % tool task

mission.previous_action = mission.actions.A1;
mission.current_action = mission.actions.A1;

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
% uvms.q 
% Initial joint positions. You can change these values to initialize the simulation with a 
% different starting position for the arm
uvms.q = [-0.0031 0 0.0128 -1.2460 0.0137 0.0853-pi/2 0.0137]'; 
% uvms.p
% initial position of the vehicle
% the vector contains the values in the following order
% [x y z r(rot_x) p(rot_y) y(rot_z)]
% RPY angles are applied in the following sequence
% R(rot_x, rot_y, rot_z) = Rz (rot_z) * Ry(rot_y) * Rx(rot_x)
% uvms.p = [8.5 38.5 -38   0 -0.06 0.5]'; 
uvms.p = [10.5 35.5 -36    0 -0.06 pi/2]'; % near rock
% uvms.p = [48.5 11.5 -33    0 -0.06 pi/2]'; % actions position
% defines the goal position for the end-effector/tool position task
uvms.goalPosition = [12.2025   37.3748  -39.8860]';
uvms.wRg = rotation(0, pi, pi/2);
uvms.wTg = [uvms.wRg uvms.goalPosition; 0 0 0 1];

% uvms.goalPositionV = [12.2025   37.3748  -39.8860+1.5]'; % normal
% uvms.goalPositionV = [12.2025 37.3748 -39.8860]';   % inside the seafloor
% uvms.goalPositionV = [12.2025 37.3748 -38]';    % far from the seafloor
% uvms.goalPositionV = [50 -12.5 -33]';   % point to begin landing 
uvms.goalPositionV = [10.75   37.3748  -38]';   % landing in front of the rock
% uvms.goalPositionV = [13.75   38  -38]';   % landing near the rock
uvms.wRgV = rotation(0, pi/6, 0);
uvms.wTgV = [uvms.wRgV uvms.goalPositionV; 0 0 0 1];

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
    
    
    
    
    % main kinematic algorithm initialization
    % ydotbar order is [qdot_1, qdot_2, ..., qdot_7, xdot, ydot, zdot, omega_x, omega_y, omega_z]
    % the vector of the vehicle linear and angular velocities are assumed
    % projected on <v>
    
    ydotbar = zeros(13,1);
    Qp = eye(13); 
    % add all the other tasks here!
    % the sequence of iCAT_task calls defines the priority
    
    
   
    % Vehicle attitude towards rock task
    [Qp, ydotbar] = iCAT_task(uvms.A.vra,       uvms.Jra,     Qp, ydotbar, uvms.xdot.vra,       0.0001,   0.01, 10);   %t7
    
    % Vehicle landing task 
    [Qp, ydotbar] = iCAT_task(uvms.A.vland,     uvms.Jva,     Qp, ydotbar, uvms.xdot.vvland,    0.0001,   0.01, 10);   %t6
    
    % Vehicle altitude task 
    [Qp, ydotbar] = iCAT_task(uvms.A.va,        uvms.Jva,     Qp, ydotbar, uvms.xdot.vva,       0.0001,   0.01, 10);   %t5
    
    % tool manipulator task
    [Qp, ydotbar] = iCAT_task(uvms.A.t,         uvms.Jt,      Qp, ydotbar, uvms.xdot.t,         0.0001,   0.01, 10); %t4
    
    % Horizontal attitude task
    [Qp, ydotbar] = iCAT_task(uvms.A.ha,        uvms.Jha,     Qp, ydotbar, uvms.xdot.vha,       0.0001,   0.01, 10);   %t3
    
    % Vehicle position and attitude task
    [Qp, ydotbar] = iCAT_task(uvms.A.vpos,      uvms.Jvpos,   Qp, ydotbar, uvms.xdot.vpos,      0.0001,   0.01, 10);    %t2
    [Qp, ydotbar] = iCAT_task(uvms.A.vatt,      uvms.Jvatt,   Qp, ydotbar, uvms.xdot.vatt,      0.0001,   0.01, 10);    %t1
    
    % last task
    [Qp, ydotbar] = iCAT_task(eye(13),     eye(13),    Qp, ydotbar, zeros(13,1),  0.0001,   0.01, 10);    % this task should be the last one
    
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
%         sensorDistance = uvms.sensorDistance %#ok<*NASGU>
        phase = mission.phase
%         altitude = uvms.w_d
%         activation_altitude = uvms.A.va
%         uvms.v_rho
        misalign_rock = norm(uvms.v_rho_rock)
        rock_activ = uvms.A.vra 
        ha = 0.1-norm(uvms.v_rho)
        activ = uvms.A.ha
        %         [~, w_vlin] = CartError(uvms.wTgV , uvms.wTv);
        %         norm(w_vlin)
        %         activation_horiz = uvms.A.ha
    end

    % enable this to have the simulation approximately evolving like real
    % time. Remove to go as fast as possible
    SlowdownToRealtime(deltat);
end

fclose(uVehicle);
fclose(uArm);

% PrintPlot(plt);

end