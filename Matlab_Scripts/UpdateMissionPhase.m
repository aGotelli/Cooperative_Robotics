function [uvms, mission] = UpdateMissionPhase(uvms, mission)
    
    switch mission.phase
        case 1
            [~, w_vlin] = CartError(uvms.wTg_v , uvms.wTv);
            
            if (norm(w_vlin) < 0.1)
                
                mission.phase = 2;
                mission.phase_time = 0;
                mission.switch = 1;
                mission.previousAction = mission.currentAction;
                
                %% Exercise 3
%                 mission.currentAction = mission.actionLanding;
                
                %% Moving the vehicle and then the tool frame
%                 mission.currentAction = mission.toolControl;

                %% Constrained task (underactuated)
%                 mission.currentAction = mission.toolControlUnderactuated;

                %% Exercise 4
                mission.currentAction = mission.alignedLanding;
                
            end
            
        case 2
            
    end
end

