function runValkyrieTurning270()

path_handle = addpathTemporary(fullfile(getDrakePath,'examples','Valkyrie'));
runValkyrieWalking([], struct('initial_pose', [1;0;0;0;0;pi/2],...
                                'navgoal', [0;-1;0;0;0;-pi/2-pi/16],...
                                'max_num_steps', 15));
end

% TIMEOUT 1000