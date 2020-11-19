function [uvms, mission] = UpdateMissionPhase_ex1(uvms, mission)
    switch mission.phase
        case 1  
            [~, w_vlin] = CartError(uvms.wTg_v , uvms.wTv);
            
            if (norm(w_vlin) < 0.1)
                
                mission.phase = 2;
%                 mission.phase_time = 0;
%                 mission.switch = 1;
%                 mission.previousAction = mission.currentAction;
                
            end
            
        case 2 
            
    end
end