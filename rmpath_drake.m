function rmpath_drake

try 
  load drake_config.mat;
catch
  disp('drake doesn''t appear to be in your path');
  return;
end

rmpath(fullfile(conf.root,'systems'));
rmpath(fullfile(conf.root,'systems','plants'));
rmpath(fullfile(conf.root,'systems','plants','affordance'));
rmpath(fullfile(conf.root,'systems','plants','collision'));
rmpath(fullfile(conf.root,'systems','plants','constraint'));
rmpath(fullfile(conf.root,'systems','plants','trajOpt'));
rmpath(fullfile(conf.root,'systems','controllers'));
rmpath(fullfile(conf.root,'systems','observers'));
rmpath(fullfile(conf.root,'systems','trajectories'));
rmpath(fullfile(conf.root,'systems','frames'));
rmpath(fullfile(conf.root,'systems','visualizers'));
rmpath(fullfile(conf.root,'systems','robotInterfaces'));
rmpath(fullfile(conf.root,'solvers'));
rmpath(fullfile(conf.root,'util'));
rmpath(fullfile(conf.root,'util','obstacles'));
rmpath(fullfile(conf.root,'thirdParty'));
rmpath(fullfile(conf.root,'thirdParty','path'));
rmpath(fullfile(conf.root,'thirdParty','spatial'));
rmpath(fullfile(conf.root,'thirdParty','cprintf'));
rmpath(fullfile(conf.root,'thirdParty','GetFullPath'));

javarmpath(fullfile(pods_get_base_path,'share','java','drake.jar'));
javarmpath(fullfile(pods_get_base_path,'share','java','lcmtypes_drake.jar'));
