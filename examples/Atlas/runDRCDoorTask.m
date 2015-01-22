function runDRCDoorTask

checkDependency('iris');
checkDependency('mosek');

options.floating = true;
options.replace_cylinders_with_capsules = false;
options.atlas_version = 4;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

door = RigidBodyManipulator();
door = door.setTerrain(RigidBodyFlatTerrain());
door = door.addRobotFromURDF('urdf/door.urdf',[1;0;0],[0;0;pi],options);
x0_door = Point(door.getStateFrame(), door.getInitialState());
x0_door.hinge = pi/2;

heightmap = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(door,x0_door(1:door.getNumPositions()),-1:.02:3,-1:.02:1,10);

options.initial_pose = [0;0;0;0;0;pi/2];
options.terrain = heightmap;

region_server = iris.terrain_grid.Server();
region_server.addHeightmap(1, options.terrain);

region_args = {r.getFootstepPlanningCollisionModel(), ...
   'xy_bounds', iris.Polytope.fromBounds([-1,-.5], [3, .5]),...
   'debug', false};

i = 1;
while true
%   profile on
  options.navgoal = [options.initial_pose(1)+3; 0;0;0;0;0];
  if isfield(options, 'x0')
    feet = r.feetPosition(options.x0(1:r.getNumPositions()));
    seeds = [feet.right, feet.left];
  else
    seeds = [];
  end
  options.safe_regions = region_server.findSafeTerrainRegions(1, region_args{:}, 'seeds', [1;0;0;0;0;pi/2]);
  options.safe_regions(end+1) = region_server.getCSpaceRegionAtIndex(region_server.xy2ind(1, options.initial_pose(1:2)), options.initial_pose(6), region_args{:});
  figure(i+20)
  i = i + 1;
  clf
  hold on
  for j = 1:length(options.safe_regions)
    iris.drawing.drawPolyFromVertices(iris.thirdParty.polytopes.lcon2vert(options.safe_regions(j).A, options.safe_regions(j).b)', 'r');
  end
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
  options.initial_pose;
end




% x0 = r.resolveConstraints(x0);

%options.use_collision_geometry = true;
% v = r.constructVisualizer(options);
% %v.drawWrapper(0,x0);
% v.inspector(x0)