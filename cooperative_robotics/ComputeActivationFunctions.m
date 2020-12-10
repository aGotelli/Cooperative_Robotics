function [uvms] = ComputeActivationFunctions(uvms, mission)
% compute the activation functions here

%% Mission
% When we execute the first action we need to activate the corresponding
% tasks
if isempty(mission.previousAction)
    for task = 1:size(mission.currentAction, 2)
        dimension = size(mission.activationFunctions{mission.currentAction(task)}, 1);
        mission.activationFunctions{mission.currentAction(task)} = eye(dimension);
    end
% When we need to perform a new action we need to change the activation
% functions
elseif mission.switch == 1        
    for task = 1:mission.totalNumOfTasks
        % Size of the activation function of the task
        dimension = size(mission.activationFunctions{task}, 1);
        
        % If the task is both in the previous and current action the act.
        % fun. is the identity
        if ~isempty(find(mission.previousAction == task, 1)) && ~isempty(find(mission.currentAction == task, 1))
            mission.activationFunctions{task} = eye(dimension);
        % If the task is in the previous action but not in the current one 
        % then the act. fun. is decreasing
        elseif ~isempty(find(mission.previousAction == task, 1)) && isempty(find(mission.currentAction == task, 1))
            mission.activationFunctions{task} = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
        % If the task isn't in the previous action but is in the current 
        % one then the act. fun. is increasing
        elseif isempty(find(mission.previousAction == task, 1)) && ~isempty(find(mission.currentAction == task, 1))
            mission.activationFunctions{task} = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time);
        % In this case the task is neither in the previous action nor in
        % the current one
        else
            mission.activationFunctions{task} = zeros(dimension);
        end
    end
    
    mission.switch = 0;
end

%% Arm tool position control
% always active
uvms.A.t = eye(6) * mission.activationFunctions{1};

%%   HORIZONTAL ATTITUDE CONTROL
uvms.A.ha = IncreasingBellShapedFunction(0.01, 0.05, 0, 1, norm(uvms.v_rho)) * mission.activationFunctions{2};

%%   ATTITUDE AND POSITION CONTROL
uvms.A.v_pos = eye(3) * mission.activationFunctions{3};
uvms.A.v_att = eye(3) * mission.activationFunctions{4};

%%   ACTIVATION FUNCTION FOR THE TASK ENSURING OFFSET 
uvms.A.minAlt = DecreasingBellShapedFunction(uvms.min_offset, (uvms.min_offset + uvms.range_offset), 0, 1, uvms.w_a) * mission.activationFunctions{5};

%%   ACTIVATION FUNCTION FOR THE LANDING TASK
uvms.A.landing = eye(1) * mission.activationFunctions{6};

%%   ACTIVATION FUNCTION FOR THE HORIZONTAL ALIGNMENT TO TARGET
uvms.A.horAlign = eye(1) * mission.activationFunctions{7};

%%   ACTIVATION FUNCTION FOR ENSURING THE GOAL IN THE ARM WORKSPACE
uvms.A.distGoal = diag([IncreasingBellShapedFunction(uvms.ensured_distance_x, 2, 0, 1, uvms.xd),...
                        IncreasingBellShapedFunction(uvms.ensured_distance_y, 1, 0, 1, uvms.yd)]).* mission.activationFunctions{8};
               
%%   ACTIVATION FUNCTION FOR CONTRAINING THE VELOCITIES
uvms.A.constraint = eye(6) * mission.activationFunctions{9};

%%  ACTIVATION FUNCTION FOR AVOIDING LOWER BOUND JOINT LIMITS
offset = 0.5;
uvms.A.lbJointLimits = diag([DecreasingBellShapedFunction(uvms.jlmin(1), uvms.jlmin(1) + offset, 0, 1, uvms.q(1)),...
                             DecreasingBellShapedFunction(uvms.jlmin(2), uvms.jlmin(2) + offset, 0, 1, uvms.q(2)),...
                             DecreasingBellShapedFunction(uvms.jlmin(3), uvms.jlmin(3) + offset, 0, 1, uvms.q(3)),...
                             DecreasingBellShapedFunction(uvms.jlmin(4), uvms.jlmin(4) + offset, 0, 1, uvms.q(4)),...
                             DecreasingBellShapedFunction(uvms.jlmin(5), uvms.jlmin(5) + offset, 0, 1, uvms.q(5)),...
                             DecreasingBellShapedFunction(uvms.jlmin(6), uvms.jlmin(6) + offset, 0, 1, uvms.q(6)),...
                             DecreasingBellShapedFunction(uvms.jlmin(7), uvms.jlmin(7) + offset, 0, 1, uvms.q(7))]) .* mission.activationFunctions{10};

%%  ACTIVATION FUNCTION FOR AVOIDING UPPER BOUND JOINT LIMITS
uvms.A.ubJointLimits = diag([IncreasingBellShapedFunction(uvms.jlmax(1) - offset, uvms.jlmax(1), 0, 1, uvms.q(1)),...
                             IncreasingBellShapedFunction(uvms.jlmax(2) - offset, uvms.jlmax(2), 0, 1, uvms.q(2)),...
                             IncreasingBellShapedFunction(uvms.jlmax(3) - offset, uvms.jlmax(3), 0, 1, uvms.q(3)),...
                             IncreasingBellShapedFunction(uvms.jlmax(4) - offset, uvms.jlmax(4), 0, 1, uvms.q(4)),...
                             IncreasingBellShapedFunction(uvms.jlmax(5) - offset, uvms.jlmax(5), 0, 1, uvms.q(5)),...
                             IncreasingBellShapedFunction(uvms.jlmax(6) - offset, uvms.jlmax(6), 0, 1, uvms.q(6)),...
                             IncreasingBellShapedFunction(uvms.jlmax(7) - offset, uvms.jlmax(7), 0, 1, uvms.q(7))]) .* mission.activationFunctions{11};
                         
end
