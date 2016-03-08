function rmpath_drake

root = getDrakePath();

if ispc
  p = getenv('PATH');
  warning('Drake lib directories have been added to your system path and are not removed (yet)');
  % todo: remove added paths
end

rmpath(fullfile(root,'systems'));
rmpath(fullfile(root,'systems','plants'));
rmpath(fullfile(root,'systems','plants','collision'));
rmpath(fullfile(root,'systems','plants','constraint'));
rmpath(fullfile(root,'systems','controllers'));
rmpath(fullfile(root,'systems','observers'));
rmpath(fullfile(root,'systems','trajectories'));
rmpath(fullfile(root,'systems','trajectories','TrajectoryLibraries'));
rmpath(fullfile(root,'systems','trajectories','FunnelLibraries'));
rmpath(fullfile(root,'systems','frames'));
rmpath(fullfile(root,'systems','visualizers'));
rmpath(fullfile(root,'systems','robotInterfaces'));
rmpath(fullfile(root,'systems','robotInterfaces','calibration'));
rmpath(fullfile(root,'solvers'));
rmpath(fullfile(root,'solvers','trajectoryOptimization'));
rmpath(fullfile(root,'util'));
rmpath(fullfile(root,'util','geometry'));
rmpath(fullfile(root,'util','visualization'));
rmpath(fullfile(root,'thirdParty'));
rmpath(fullfile(root,'thirdParty','psm'));
rmpath(fullfile(root,'thirdParty','path'));
rmpath(fullfile(root,'thirdParty','cprintf'));
rmpath(fullfile(root,'thirdParty','GetFullPath'));
rmpath(fullfile(root,'thirdParty','plotregion'));
rmpath(fullfile(root,'thirdParty','polytopes'));
rmpath(fullfile(root,'solvers','BMI'));
rmpath(fullfile(root,'solvers','BMI','util'));
rmpath(fullfile(root,'solvers','BMI','kinematics'));
rmpath(fullfile(root,'solvers','qpSpline'));
rmpath(fullfile(root,'bindings','matlab'));

javarmpath(fullfile(pods_get_base_path,'share','java','drake.jar'));
javarmpath(fullfile(pods_get_base_path,'share','java','lcmtypes_drake.jar'));
