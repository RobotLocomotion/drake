function testSplitWalking()
checkDependency('lcm');
path_handle = addpathTemporary(fullfile(getDrakePath(), 'examples', 'Atlas'));

% Run a short plan through both modes to make sure they work:
runAtlasWalkingSplit(struct('use_mex', 0, 'navgoal', [0.2;0;0;0;0;0], 'num_steps', 1));
runAtlasWalkingSplit(struct('use_mex', 1, 'navgoal', [0.2;0;0;0;0;0], 'num_steps', 1));

% Then a longer plan, comparing mex and non-mex outputs
runAtlasWalkingSplit(struct('use_mex', 2));

%TIMEOUT 1500