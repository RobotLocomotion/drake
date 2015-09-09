function runDRCDoorTask
% Plan a walking trajectory to pass through a simulated doorframe. 

checkDependency('iris');
checkDependency('mosek');

options.floating = true;
options.replace_cylinders_with_capsules = false;
options.atlas_version = 3;
options.step_params.max_num_steps = 14;
r = Atlas('urdf/atlas_minimal_contact.urdf',options);

door = RigidBodyManipulator();
door = door.setTerrain(RigidBodyFlatTerrain());
door = door.addRobotFromURDF('urdf/door.urdf',[1;0;0],[0;0;pi],options);
x0_door = Point(door.getStateFrame(), door.getInitialState());
x0_door.hinge = pi/2;

heightmap = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(door,x0_door(1:door.getNumPositions()),-1:.015:3,-1:.015:1,10);
r = r.setTerrain(heightmap).compile();

options.initial_pose = [0;0;0;0;0;pi/2];
options.terrain = heightmap;

region_server = iris.terrain_grid.Server();
region_server.addHeightmap(1, options.terrain);

region_args = {r.getFootstepPlanningCollisionModel(), ...
   'xy_bounds', iris.Polyhedron.fromBounds([-1,-.5], [3, .5]),...
   'debug', false};

i = 1;  
options.safe_regions = region_server.findSafeTerrainRegions(1, region_args{:}, 'seeds', [1;0;0;0;0;pi/2]);

combined_xtraj = [];

while true
  options.navgoal = [options.initial_pose(1)+3; 0;0;0;0;0];
  options.safe_regions(end+1) = region_server.getCSpaceRegionAtIndex(region_server.xy2ind(1, options.initial_pose(1:2)), options.initial_pose(6), region_args{:}, 'error_on_infeasible_start', false);
%   figure(i+20)
%   i = i + 1;
%   clf
%   hold on
%   for j = 1:length(options.safe_regions)
%     iris.drawing.drawPolyFromVertices(iris.thirdParty.polytopes.lcon2vert(options.safe_regions(j).A, options.safe_regions(j).b)', 'r');
%   end
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
  options.initial_pose;
end

r.constructVisualizer().playback(combined_xtraj, struct('slider', true));
rangecheck(options.initial_pose(1:2), [1;-1], [inf; 1]);
