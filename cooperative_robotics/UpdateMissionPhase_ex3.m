function [uvms, mission] = UpdateMissionPhase_ex3(uvms, mission)
    switch mission.phase
        case 1  
            [~, w_vlin] = CartError(uvms.wTg_v , uvms.wTv);
            
            if (norm(w_vlin) < 0.1)
                
                mission.phase = 2;
                mission.phase_time = 0;
                mission.switch = 1;
                mission.previousAction = mission.currentAction;
                % Exercise 3.1
                % mission.currentAction = mission.actionAlignedLanding; 
                % Exercise 3.1.3
                mission.currentAction = mission.actionAligning; 

            end
            
        case 2  
            if (uvms.theta < 0.02)
                mission.phase = 3;
                mission.phase_time = 0;
                mission.switch = 1;
                mission.previousAction = mission.currentAction;
                % Exercise 3.1.3
                mission.currentAction = mission.actionAlignedLanding; 
                
            end
        case 3
            
            
    end
end