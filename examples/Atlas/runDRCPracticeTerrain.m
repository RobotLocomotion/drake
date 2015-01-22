function runDRCPracticeTask

checkDependency('iris');
task_number=2; % consider taking the task number as an input if/when we load the other tasks

clear gazeboModelPath;
setenv('GAZEBO_MODEL_PATH',[fullfile(getDrakePath,'examples','Atlas','sdf')]); 

if exist('data/practice_terrain.mat','file')
  load data/practice_terrain.mat;
else
  terrain = RigidBodyManipulator(['sdf/drc_practice_task_',num2str(task_number),'.world']);
  height_map = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(terrain,[],-3:.02:10,-3:.02:3,10);
  save data/practice_terrain.mat height_map;
end

options.terrain = height_map;  
options.initial_pose = [-3;0;0;0;0;0];
% options.initial_pose = [0;0;0;0;0;0]
% options.initial_pose = [-1;0;0;0;0;pi/4];

region_server = iris.terrain_grid.Server();
region_server.addHeightmap(1, options.terrain);

while true
%   profile on
  options.navgoal = [options.initial_pose(1)+3; 0;0;0;0;0];
  % options.safe_regions = region_server.findSafeTerrainRegions(1, 
  options.safe_regions = findSafeTerrain(options.terrain, options.initial_pose, options.navgoal);
  try
    xtraj = runAtlasWalkingPlanning(options);
  catch e
    if strcmp(e.identifier, 'Drake:NoFeasibleFootstepPlan')
      % then we're done
      break
    else
      rethrow(e);
    end
  end
%   profile viewer
  breaks = xtraj.getBreaks();
  xf = xtraj.eval(breaks(end));
  options.x0 = xf;
  options.initial_pose = xf(1:6);
  disp('pausing for playback...use dbcont to continue');
  keyboard()
  options.initial_pose
end




