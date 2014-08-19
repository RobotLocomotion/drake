function rmpath_drake

root = getDrakePath();

rmpath(fullfile(root,'systems'));
rmpath(fullfile(root,'systems','plants'));
rmpath(fullfile(root,'systems','plants','affordance'));
rmpath(fullfile(root,'systems','plants','collision'));
rmpath(fullfile(root,'systems','plants','constraint'));
rmpath(fullfile(root,'systems','controllers'));
rmpath(fullfile(root,'systems','observers'));
rmpath(fullfile(root,'systems','trajectories'));
rmpath(fullfile(root,'systems','frames'));
rmpath(fullfile(root,'systems','visualizers'));
rmpath(fullfile(root,'systems','robotInterfaces'));
rmpath(fullfile(root,'solvers'));
rmpath(fullfile(root,'util'));
rmpath(fullfile(root,'util','obstacles'));
rmpath(fullfile(root,'thirdParty'));
rmpath(fullfile(root,'thirdParty','path'));
rmpath(fullfile(root,'thirdParty','spatial'));
rmpath(fullfile(root,'thirdParty','cprintf'));
rmpath(fullfile(root,'thirdParty','GetFullPath'));

javarmpath(fullfile(pods_get_base_path,'share','java','drake.jar'));
javarmpath(fullfile(pods_get_base_path,'share','java','lcmtypes_drake.jar'));
