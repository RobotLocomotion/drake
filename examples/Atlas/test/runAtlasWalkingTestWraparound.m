function runAtlasWalkingTestWraparound()

path_handle = addpathTemporary(fullfile(getDrakePath,'examples','Atlas'));
options = struct();
options.initial_pose = [0;0;0;0;0;-pi+pi/16];
options.navgoal = [0;0;0;0;0;pi-pi/16];
runAtlasWalking(options);

end

