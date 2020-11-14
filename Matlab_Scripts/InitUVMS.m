function [uvms] = InitUVMS(robotname)

% uvms.vTb
% Transformation matrix between the arm base wrt vehicle frame
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

% Joint limits corresponding to the actual MARIS arm configuration
uvms.jlmin  = [-2.9;-1.6;-2.9;-2.95;-2.9;-1.65;-2.8];
uvms.jlmax  = [2.9;1.65;2.9;0.01;2.9;1.25;2.8];

% To be computed at each time step
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


%% Jacobians
% Default ones
uvms.Jjl = [];
uvms.Jmu = [];
uvms.Jt_a = [];
uvms.Jt_v = [];
uvms.Jt = [];

% Exercise 1
uvms.Jvpos = [];
uvms.Jvatt = [];

% Vehicle Hor. Att.
uvms.Jha = [];

% Exercise 2
uvms.Jalt = [];

% Constrained task (underactuated)
uvms.Jua = [];

% Exercise 4
uvms.JhorAlign = [];

%% Task references
% Default ones
uvms.xdot.jl = [];
uvms.xdot.mu = [];
uvms.xdot.t = [];

% Exercise 1
uvms.xdot.vpos = [];
uvms.xdot.vatt = [];

% Vehicle Hor. Att.
uvms.xdot.ha = [];

% Exercise 2
uvms.xdot.alt = [];

% Constrained task (underactuated)
uvms.xdot.ua = [];

% Exercise 4
uvms.xdot.horAlign = [];

%% Activation functions
% Default ones
uvms.A.jl = zeros(7);
uvms.A.mu = zeros(1);
uvms.A.t = zeros(6);

% Exercise 1
uvms.A.vpos = zeros(3);
uvms.A.vatt = zeros(3);

% Vehicle Hor. Att.
uvms.A.ha = zeros(1);

% Exercise 2
uvms.A.minAlt = zeros(1);

% Exercise 3 (mission)
uvms.A.alt = zeros(1);

% Constrained task (underactuated)
uvms.A.ua = zeros(6);

% Exercise 4
uvms.A.horAlign = zeros(1);

%% Other quantities
% Vehicle Hor. Att.
uvms.v_rho = zeros(3, 1);

% Exercise 4
uvms.theta = zeros(1);

end

