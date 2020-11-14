function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    switch mission.phase
        case 1  % action 1
            % policy for changing phase
            [~, w_vlin] = CartError(uvms.wTgV , uvms.wTv);
            if norm(w_vlin) < 0.2
               mission.phase = 2;
               mission.previous_action = mission.current_action;
               mission.current_action = mission.actions.A2;
               mission.phase_time = 0;
            end
        case 2  % action 2 landing
            if norm(uvms.w_d) < 0.15
                mission.phase = 3;
                mission.previous_action = mission.current_action;
                mission.current_action = mission.actions.A3;

                mission.phase_time = 0;
            end
        case 3
           
            
    end
end

