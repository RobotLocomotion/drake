function [xtraj, info] = testMultipleTaskSpaceRRTStar(options, rng_seed)
% TIMEOUT 600
if nargin < 1 || isempty(options), options = struct(); end
if nargin < 2, rng; else rng(rng_seed); end
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
if ~isfield(options,'goal_bias'), options.goal_bias = 0.5; end;
if ~isfield(options,'n_smoothing_passes'), options.n_smoothing_passes = 10; end;
if ~isfield(options,'planning_mode'), options.planning_mode = 'multiRRT'; end;
if ~isfield(options,'visualize'), options.visualize = true; end;
if ~isfield(options,'scene'), options.scene = 1; end;
if ~isfield(options,'model'), options.model = 'v5'; end;
if ~isfield(options,'convex_hull'), options.convex_hull = true; end;
if ~isfield(options,'graspingHand'), options.graspingHand = 'right'; end;
if ~isfield(options,'costType'), options.costType = 'length'; end;
if ~isfield(options,'firstFeasibleTraj'), options.firstFeasibleTraj = false; end;
if ~isfield(options,'robot'), options.robot = []; end;
if ~isfield(options,'nTrees'), options.nTrees = 4; end;
if ~isfield(options,'goalObject'), options.goalObject = 1; end;
options.floating = true;
options.terrain = RigidBodyFlatTerrain(); %Changed to a smaller terrain to avoid visualization problem when zooming
options.joint_v_max = 15*pi/180;
options.viewer = 'NullVisualizer';


urdf = fullfile(getDrakePath(),'../../','models','atlas_v5','model_chull.urdf');
r = RigidBodyManipulator(urdf,options);
options.fp = load([getDrakePath(), '/../../control/matlab/data/atlas_v5/atlas_v5_fp.mat']);
options.qNominalC_fp = load([getDrakePath(), '/../../control/matlab/data/atlas_v5/atlasv5_fp_rHand_up.mat']);
options.qNominalD_fp = load([getDrakePath(), '/../../control/matlab/data/atlas_v5/atlasv5_fp_rHand_up_right.mat']);

q_nom = options.fp.xstar(1:r.getNumPositions());
qNominalC = options.qNominalC_fp.xstar(1:r.getNumPositions());
qNominalD = options.qNominalD_fp.xstar(1:r.getNumPositions());

% Create Scene:
world = r.findLinkId('world');
table = RigidBodyBox([.6 1 .025], [.7 0 .9], [0 0 0]);
r = addGeometryToBody(r, world, table);
targetObject = RigidBodyBox([.05 .05 .3], [0.8 0 1.0625], [0 0 0]);
targetObject = targetObject.setColor([1 0 0]);
r = addGeometryToBody(r, world, targetObject);

r = r.compile();

lFoot = r.findLinkId('l_foot');
rFoot = r.findLinkId('r_foot');
g_hand = r.findLinkId('r_hand');
options.point_in_link_frame = [0; -0.3; 0];
options.targetObjectPos = [0.8 0 1.0625];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if options.visualize
  visWorld = RigidBodyManipulator();
  for b = 1:numel(r.body(1).visual_geometry)
    visWorld = addGeometryToBody(visWorld, 1, r.body(1).visual_geometry{b});
  end
  visWorld = visWorld.compile();
  %  v = visWorld.constructVisualizer();
  v = r.constructVisualizer();
  v.draw(0,q_nom);
end

%Set IK options
ik_seed_pose = q_nom;
ik_nominal_pose = q_nom;
cost = Point(r.getPositionFrame(),10);
for i = r.getNumBodies():-1:1
  if all(r.getBody(i).parent > 0) && all(r.getBody(r.getBody(i).parent).position_num > 0)
    cost(r.getBody(r.getBody(i).parent).position_num) = ...
      cost(r.getBody(r.getBody(i).parent).position_num) + cost(r.getBody(i).position_num);
  end
end
cost(1:6) = max(cost(7:end))/2;
cost = cost/min(cost);
Q = diag(cost);
ikoptions = IKoptions(r);
ikoptions = ikoptions.setMajorIterationsLimit(100);
ikoptions = ikoptions.setQ(Q);
ikoptions = ikoptions.setMajorOptimalityTolerance(1e-3);

%Set start pose constraints and compute starting configuration
startPoseConstraints = [fixedFeetConstraints(options, r),...
  {addQuasiStaticConstraint(options, r),...
  nonGraspingHandDistanceConstraint(options, r, 0.4)}];
[q_start, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, startPoseConstraints{:}, ikoptions);
if options.visualize
  v.draw(0,q_start);
  
end

%Set end pose constraints and compute end configuration
goalConstraints = addGoalConstraint(options, r);
endPoseConstraints = [startPoseConstraints, goalConstraints];

[q_end, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, endPoseConstraints{:}, ikoptions);
if options.visualize
  v.draw(0,q_end);
end

%Create RRTs
inactive_collision_bodies = [lFoot,rFoot];

kinsol = r.doKinematics(q_start);
xyz_quat_start = r.forwardKin(kinsol,g_hand,options.point_in_link_frame,2);
kinsol = r.doKinematics(q_end);
xyz_quat_goal = r.forwardKin(kinsol,g_hand,options.point_in_link_frame,2);
x_start = [xyz_quat_start;q_start];
x_goal = [xyz_quat_goal;q_end];


kinsol = r.doKinematics(qNominalC);
EEpose = r.forwardKin(kinsol, r.findLinkId('r_hand'),  [0; -0.3; 0], 2);
constraints = [startPoseConstraints, generateEEConstraints(r, options, EEpose)];
[qStartC, info, infeasible_constraint] = inverseKin(r, qNominalC, qNominalC, constraints{:}, ikoptions);
if options.visualize
  v.draw(0,qStartC);
end
kinsol = r.doKinematics(qStartC);
xyz_quat_start = r.forwardKin(kinsol,g_hand,options.point_in_link_frame,2);
xStartC = [xyz_quat_start; qStartC];

kinsol = r.doKinematics(qNominalD);
EEpose = r.forwardKin(kinsol, r.findLinkId('r_hand'),  [0; -0.3; 0], 2);
constraints = [startPoseConstraints, generateEEConstraints(r, options, EEpose)];
[qStartD, info, infeasible_constraint] = inverseKin(r, qNominalD, qNominalD, constraints{:}, ikoptions);
if options.visualize
  v.draw(0,qStartD);
end
kinsol = r.doKinematics(qStartD);
xyz_quat_start = r.forwardKin(kinsol,g_hand,options.point_in_link_frame,2);
xStartD = [xyz_quat_start; qStartD];

if options.visualize
   v.draw(0,q_start);
end

rrt_timer = tic;
display('About to plan ...')

x_end.val1 = options.targetObjectPos';
x_end.val2 = x_end.val1;
x_end.v5 = x_goal;

finalPose = FinalPoseProblem(r, g_hand, x_start, x_end.(options.model), ...
  startPoseConstraints, goalConstraints, q_nom, ...
  'capabilityMap', [], 'graspinghand', options.graspingHand, ...
  'activecollisionoptions', ...
  struct('body_idx', setdiff(1:r.getNumBodies(), inactive_collision_bodies)),...
  'ikoptions', ikoptions, ...
  'endeffectorpoint', options.point_in_link_frame);

optionsPlanner = struct();
if ~isfield(optionsPlanner,'costType'), optionsPlanner.costType = 'length'; end;

switch options.nTrees
  case 4
    multiTree = MultipleTreeProblem(r, g_hand, x_start, x_end.(options.model), ...
      [xStartC, xStartD], startPoseConstraints, q_nom,...
      'activecollisionoptions', ...
      struct('body_idx', setdiff(1:r.getNumBodies(), inactive_collision_bodies)),...
      'ikoptions', ikoptions, 'endeffectorpoint', options.point_in_link_frame);
    
  case 3
    multiTree = MultipleTreeProblem(r, g_hand, x_start, x_end.(options.model), ...
      [xStartC], startPoseConstraints, q_nom,...
      'activecollisionoptions', ...
      struct('body_idx', setdiff(1:r.getNumBodies(), inactive_collision_bodies)),...
      'ikoptions', ikoptions, 'endeffectorpoint', options.point_in_link_frame);
  case 2
    multiTree = MultipleTreeProblem(r, g_hand, x_start, x_end.(options.model), ...
      [], startPoseConstraints, q_nom,...
      'activecollisionoptions', ...
      struct('body_idx', setdiff(1:r.getNumBodies(), inactive_collision_bodies)),...
      'ikoptions', ikoptions, 'endeffectorpoint', options.point_in_link_frame);
end

[xGoalFull, info] = finalPose.findFinalPose(optionsPlanner);
[multiTree, info, cost, q_path] = multiTree.rrtStar(optionsPlanner, xGoalFull);

if info == 1
  path_length = size(q_path,2);
  xtraj = PPTrajectory(pchip(linspace(0, 1, path_length), [q_path(8:end,:); zeros(r.getNumVelocities(), size(q_path,2))] ));
else
  xtraj = [];
  info = 13;
end


xtraj = xtraj.setOutputFrame(r.getStateFrame());

rrt_time = toc(rrt_timer);
fprintf(['TIMING:\n',...
  '\ttotal Time: %.2f\n'],...
  rrt_time);

end



function quasiStaticConstraint = addQuasiStaticConstraint(options, robot)
l_foot = robot.findLinkId('l_foot');
r_foot = robot.findLinkId('r_foot');
l_foot_pts = robot.getBody(l_foot).getTerrainContactPoints();
r_foot_pts = robot.getBody(r_foot).getTerrainContactPoints();
quasiStaticConstraint = QuasiStaticConstraint(robot, [-inf, inf], 1);
quasiStaticConstraint = quasiStaticConstraint.setShrinkFactor(0.5);
quasiStaticConstraint = quasiStaticConstraint.setActive(true);
quasiStaticConstraint = quasiStaticConstraint.addContact(l_foot, l_foot_pts);
quasiStaticConstraint = quasiStaticConstraint.addContact(r_foot, r_foot_pts);
end

function constraints = fixedFeetConstraints(options, robot, state, foot)
if nargin < 4, foot = 'both'; end;
if nargin < 3
  state = options.fp.xstar(1:robot.getNumPositions());
end
leftConstraints = {};
rightConstraints = {};
kinsol = robot.doKinematics(state);
l_foot = robot.findLinkId('l_foot');
r_foot = robot.findLinkId('r_foot');
if any(strcmp(foot, {'both', 'left'}))
  footPose = robot.forwardKin(kinsol,l_foot, [0; 0; 0], 2);
  %                 drawLinkFrame(robot, l_foot, state, 'Left Foot Frame');
  leftFootPosConstraint = WorldPositionConstraint(robot, l_foot, [0; 0; 0], footPose(1:3), footPose(1:3));
  leftFootQuatConstraint = WorldQuatConstraint(robot, l_foot, footPose(4:7), 0.0, [0.0, 1.0]);
  leftConstraints = {leftFootPosConstraint, leftFootQuatConstraint};
end
if any(strcmp(foot, {'both', 'right'}))
  footPose = robot.forwardKin(kinsol,r_foot, [0; 0; 0], 2);
  %                 drawLinkFrame(robot, r_foot, state, 'Right Foot Frame');
  rightFootPosConstraint = WorldPositionConstraint(robot, r_foot, [0; 0; 0], footPose(1:3), footPose(1:3));
  rightFootQuatConstraint = WorldQuatConstraint(robot, r_foot, footPose(4:7), 0.0, [0.0, 1.0]);
  rightConstraints = {rightFootPosConstraint, rightFootQuatConstraint};
end
constraints = [leftConstraints, rightConstraints];
end


function constraint = nonGraspingHandDistanceConstraint(options, robot, dist)
elbow = robot.findLinkId('l_larm');
trunk = robot.findLinkId('utorso');
constraint = Point2PointDistanceConstraint(robot, elbow, trunk, [0; 0; 0], [0; 0; 0], dist, Inf);
end


function constraints = addGoalConstraint(options, robot)
hand = robot.findLinkId('r_hand');
goalFrame = [eye(3) options.targetObjectPos'; 0 0 0 1];
goalEulerConstraint.left.v5 = WorldEulerConstraint(robot, hand, [-pi; pi/2;0 ], [pi; pi/2; 0]);
goalEulerConstraint.right.v5 = WorldEulerConstraint(robot, hand, [-pi; -pi/2;0 ], [pi; -pi/2; 0]);
goalEulerConstraint.left.val1 = WorldEulerConstraint(robot, hand, [-pi/2;0; -pi], [-pi/2; 0; pi]);
goalEulerConstraint.right.val1 = goalEulerConstraint.left.val1;
goalEulerConstraint.left.val2 = WorldEulerConstraint(robot, hand, [0;0; -pi], [0; 0; pi]);
goalEulerConstraint.right.val2 = goalEulerConstraint.left.val2;
goalDistConstraint = Point2PointDistanceConstraint(robot, hand, robot.findLinkId('world'), options.point_in_link_frame, goalFrame(1:3, 4), -0.001, 0.001);
constraints = {goalDistConstraint, goalEulerConstraint.(options.graspingHand).(options.model)};
end


function constraints = generateEEConstraints(robot, options, x)
constraints = {};
xyz = x(1:3);
quat = x(4:7);
hand = robot.findLinkId('r_hand');
constraints{1} = WorldPositionConstraint(robot, hand, [0; -0.3; 0], xyz, xyz);
constraints{2} = WorldQuatConstraint(robot, hand, quat, 10*pi/180);
end