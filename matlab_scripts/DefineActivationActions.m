function [activ] = DefineActivationActions(mission,references_dim)
% generalised Activation of actions

for i=1:size(mission.current_action,2)
    
        if mission.previous_action(i) == 0 && mission.current_action(i) == 0
            activ(i).Aa =  zeros(references_dim(i));
        end
        if mission.previous_action(i) == 0 && mission.current_action(i) == i
            activ(i).Aa = IncreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time)*eye(references_dim(i));
        end
        if mission.previous_action(i) == i && mission.current_action(i) == 0
            activ(i).Aa = DecreasingBellShapedFunction(0, 2, 0, 1, mission.phase_time)*eye(references_dim(i));
        end
        if mission.previous_action(i) == i && mission.current_action(i) == i
            activ(i).Aa = eye(references_dim(i));
        end
end
end

