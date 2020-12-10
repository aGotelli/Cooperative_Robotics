function [uvms, mission] = UpdateMissionPhase_ex4(uvms, mission)
    switch mission.phase
        case 1  
            [~, w_vlin] = CartError(uvms.wTg_v , uvms.wTv);
            
            if (norm(w_vlin) < 0.1)
                
                mission.phase = 2;
                mission.phase_time = 0;
                mission.switch = 1;
                mission.previousAction = mission.currentAction;
                % Exercise 4.1
                mission.currentAction = mission.actionAligning; 

            end
            
        case 2  
            if (uvms.theta < 0.02 && uvms.dist_to_goal_proj < 2)
%             if (uvms.dist_to_goal_proj < 2) % (To screw up the robot)
                mission.phase = 3;
                mission.phase_time = 0;
                mission.switch = 1;
                mission.previousAction = mission.currentAction;
                % Exercise 4.1
                mission.currentAction = mission.actionAlignedLanding; 
                
            end
        case 3
            if (uvms.w_a < 0.05)
                mission.phase = 4;
                mission.phase_time = 0;
                mission.switch = 1;
                mission.previousAction = mission.currentAction;
                % Exercise and 4.1
                mission.currentAction = mission.actionGraspObject; 
            end
            
        case 4
    end
end