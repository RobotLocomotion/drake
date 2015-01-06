function runDRCPracticeTask

task_number=2; % consider taking the task number as an input if/when we load the other tasks

clear gazeboModelPath;
setenv('GAZEBO_MODEL_PATH',[fullfile(getDrakePath,'examples','Atlas','sdf')]); 

if exist('practice_terrain.mat','file')
  load practice_terrain.mat;
else
  terrain = RigidBodyManipulator(['sdf/drc_practice_task_',num2str(task_number),'.world']);
  height_map = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(terrain,[],-3:.02:10,-3:.02:3,10);
  save practice_terrain.mat height_map;
end

options.terrain = height_map;  
options.navgoal = [6.5;0;0;0;0;0];
options.initial_pose = [-3;0;0;0;0;0];
% options.initial_pose = [0;0;0;0;0;0]

while true
  options.safe_regions = findSafeTerrain(options.terrain, options.initial_pose, options.navgoal);
  xtraj = runAtlasWalkingPlanning(options);
  breaks = xtraj.getBreaks();
  xf = xtraj.eval(breaks(end));
  options.initial_pose = xf(1:6);
  disp('pausing for playback...press any key to continue');
  options.initial_pose
end




