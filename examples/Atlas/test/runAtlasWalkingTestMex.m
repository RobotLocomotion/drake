function runAtlasWalkingTestMex()

path_handle = addpathTemporary(fullfile(getDrakePath,'examples','Atlas'));
options = struct();
options.use_mex = 2;
options.navgoal = [0.5;0;0;0;0;0];
runAtlasWalking(options);

end

