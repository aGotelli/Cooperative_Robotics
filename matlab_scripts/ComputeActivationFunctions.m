function [uvms] = ComputeActivationFunctions(uvms, mission)

references_dim = [ length(uvms.xdot.vatt) length(uvms.xdot.vpos) length(uvms.xdot.vha) length(uvms.xdot.t)...
    length(uvms.xdot.vva) length(uvms.xdot.vvland) length(uvms.xdot.vra)];

[activ] = DefineActivationActions(mission,references_dim);
 
% compute the activation functions here
% arm tool position control
% always active
uvms.A.t = eye(6)*activ(4).Aa;  % t4

% veichle orientation 
% always active
uvms.A.vatt = eye(3)*activ(1).Aa;   %t1
uvms.A.vpos = eye(3)*activ(2).Aa;   %t2

% Horizontal attitude
% Activation function
uvms.A.ha = IncreasingBellShapedFunction(0.01, 0.1, 0, 1, norm(uvms.v_rho))*activ(3).Aa; %t3

% Vehicle altitude 
uvms.A.va = DecreasingBellShapedFunction(1, 2, 0, 1, uvms.w_d)*activ(5).Aa; %t5

% Vehicle altitude for landing
uvms.A.vland = eye(1)*activ(6).Aa; %t6

% Vehicle attitude towards rock
% uvms.A.vra =  eye(1)*activ(7).Aa; %t7
uvms.A.vra =  IncreasingBellShapedFunction(0.01, 0.1, 0, 1, norm(uvms.v_rho_rock))*activ(7).Aa; %t7
end