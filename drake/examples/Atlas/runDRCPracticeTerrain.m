function runDRCPracticeTerrain
% Plan a walking trajectory over a set of cinderblocks, like those seen at the DRC Trials in 2013. 

checkDependency('iris');
checkDependency('mosek');
task_number=2; % consider taking the task number as an input if/when we load the other tasks
options.atlas_version = 5;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

clear gazeboModelPath;
setenv('GAZEBO_MODEL_PATH',fullfile(getDrakePath,'examples','Atlas','sdf')); 

terrain = RigidBodyManipulator(['sdf/drc_practice_task_',num2str(task_number),'.world']);
terrain.constructVisualizer();
height_map = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(terrain,[],-3:.015:10,-2:.015:2,10);

r = r.setTerrain(height_map).compile();
options.terrain = height_map;  
options.initial_pose = [-3;0;0;0;0;0];
% options.initial_pose = [0;0;0;0;0;0]
% options.initial_pose = [-1;0;0;0;0;pi/4];

region_server = iris.terrain_grid.Server();
region_server.addHeightmap(1, options.terrain);

region_args = {r.getFootstepPlanningCollisionModel()};
seeds = [[-0.55;-0.08;0.15;0;0;-pi/4]];
i = 1;
combined_xtraj = [];
while true
  options.navgoal = [options.initial_pose(1)+3; 0;0;0;0;0];
  options.safe_regions = region_server.findSafeTerrainRegions(1, region_args{:}, 'xy_bounds', iris.Polyhedron.fromBounds(options.initial_pose(1:2) - [0.5;1], options.initial_pose(1:2) + [3; 1]), 'seeds', seeds);
  seeds = [];
  figure(i+20)
  i = i + 1;
  clf
  hold on
  for j = 1:length(options.safe_regions)
    iris.drawing.drawPolyFromVertices(iris.thirdParty.polytopes.lcon2vert(options.safe_regions(j).A, options.safe_regions(j).b)', 'r');
  end
  drawnow()
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
  breaks = xtraj.getBreaks();
  xf = xtraj.eval(breaks(end));
  options.x0 = xf;
  options.initial_pose = xf(1:6);
  if isempty(combined_xtraj)
    combined_xtraj = xtraj;
  else
    b = combined_xtraj.getBreaks();
    xtraj = xtraj.shiftTime(b(end)-breaks(1));
    combined_xtraj = combined_xtraj.append(xtraj);
  end
  % disp('pausing for playback...use dbcont to continue');
  % keyboard()
  options.initial_pose
end

r.constructVisualizer().playback(combined_xtraj, struct('slider', true));
rangecheck(options.initial_pose(1:2), [4;-1.5], [inf; 1.5]);

%TIMEOUT 3000
