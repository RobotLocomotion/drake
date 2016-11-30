function rmpath_drake

root = getDrakePath();

if ispc
  p = getenv('PATH');
  warning('Drake lib directories have been added to your system path and are not removed (yet)');
  % todo: remove added paths
end

rmpath(fullfile(root,'matlab','systems'));
rmpath(fullfile(root,'matlab','systems','plants'));
rmpath(fullfile(root,'matlab','systems','plants','collision'));
rmpath(fullfile(root,'matlab','systems','plants','constraint'));
rmpath(fullfile(root,'matlab','systems','controllers'));
rmpath(fullfile(root,'matlab','systems','observers'));
rmpath(fullfile(root,'matlab','systems','trajectories'));
rmpath(fullfile(root,'matlab','systems','trajectories','TrajectoryLibraries'));
rmpath(fullfile(root,'matlab','systems','trajectories','FunnelLibraries'));
rmpath(fullfile(root,'matlab','systems','frames'));
rmpath(fullfile(root,'matlab','systems','visualizers'));
rmpath(fullfile(root,'matlab','systems','robotInterfaces'));
rmpath(fullfile(root,'matlab','systems','robotInterfaces','calibration'));
rmpath(fullfile(root,'matlab','solvers'));
rmpath(fullfile(root,'matlab','solvers','trajectoryOptimization'));
rmpath(fullfile(root,'thirdParty'));
rmpath(fullfile(root,'thirdParty','bsd'));
rmpath(fullfile(root,'thirdParty','bsd','arrow3d'));
rmpath(fullfile(root,'thirdParty','bsd','cprintf'));
rmpath(fullfile(root,'thirdParty','bsd','GetFullPath'));
rmpath(fullfile(root,'thirdParty','bsd','plotregion'));
rmpath(fullfile(root,'thirdParty','bsd','polytopes'));
rmpath(fullfile(root,'thirdParty','bsd','psm'));
rmpath(fullfile(root,'thirdParty','bsd','xacro'));
rmpath(fullfile(root,'thirdParty','misc'));
rmpath(fullfile(root,'thirdParty','misc','pathlcp'));
rmpath(fullfile(root,'thirdParty','zlib'));
rmpath(fullfile(root,'matlab','solvers','BMI'));
rmpath(fullfile(root,'matlab','solvers','BMI','util'));
rmpath(fullfile(root,'matlab','solvers','BMI','kinematics'));
rmpath(fullfile(root,'matlab','solvers','qpSpline'));
rmpath(fullfile(root,'matlab','util'));
rmpath(fullfile(root,'matlab','util','geometry'));
rmpath(fullfile(root,'matlab','util','visualization'));
rmpath(fullfile(root,'bindings','matlab'));

javarmpath(fullfile(pods_get_base_path,'share','java','drake.jar'));
javarmpath(fullfile(pods_get_base_path,'share','java','lcmtypes_drake.jar'));
