function runDRCDoorTask
% doesn't actual demonstrate any functionality yet.  just a placeholder for now 

tmppath = addpathTemporary(fullfile(pwd,'..'));

options.floating = true;
options.replace_cylinders_with_capsules = false;
r = RigidBodyManipulator('urdf/atlas_minimal_contact.urdf',options);  % note: will need more contact for this
r = r.addRobotFromURDF('urdf/door.urdf',[1;0;0],[0;0;pi],options);

door = RigidBodyManipulator();
door = door.setTerrain(RigidBodyFlatTerrain());
door = door.addRobotFromURDF('urdf/door.urdf',[1;0;0],[0;0;pi],options);
x0_door = Point(door.getStateFrame(), door.getInitialState());
x0_door.hinge = pi/2;

heightmap = RigidBodyHeightMapTerrain.constructHeightMapFromRaycast(door,x0_door(1:door.getNumPositions()),-2:.02:2,-2:.02:2,10);

load('data/atlas_fp.mat', 'xstar');
x0 = mergeCoordinates(getStateFrame(r),{xstar,x0_door});

options.initial_pose = [0;0;0;0;0;pi/2];
options.terrain = heightmap;

while true
%   profile on
  options.navgoal = [options.initial_pose(1)+3; 0;0;0;0;0];
  options.safe_regions = findSafeTerrain(heightmap, options.initial_pose, options.navgoal,...
    'width', 1.5, ...
    'xy_bounds', iris.Polytope.fromBounds([-1,-.5], [3, .5]),...
    'seeds', [1;0;0;0;0;pi/2]);
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
%   keyboard()
  options.initial_pose
end




% x0 = r.resolveConstraints(x0);

%options.use_collision_geometry = true;
% v = r.constructVisualizer(options);
% %v.drawWrapper(0,x0);
% v.inspector(x0)