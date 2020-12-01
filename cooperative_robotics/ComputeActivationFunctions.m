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
uvms.A.distGoal = IncreasingBellShapedFunction(uvms.ensured_distance, 1.0, 0, 1, uvms.dist_to_goal_proj) * mission.activationFunctions{8};


end
