function [xtraj, info] = testMultipleTaskSpaceRRTStar(options, rng_seed)
% TIMEOUT 600
if nargin < 1 || isempty(options), options = struct(); end
if nargin < 2, rng; else rng(rng_seed); end
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
if ~isfield(options,'goal_bias'), options.goal_bias = 0.5; end;
if ~isfield(options,'n_smoothing_passes'), options.n_smoothing_passes = 10; end;
if ~isfield(options,'visualize'), options.visualize = true; end;

options.floating = true;
options.terrain = RigidBodyFlatTerrain();
options.joint_v_max = 15*pi/180;

urdf = fullfile(getDrakePath(),'examples','Atlas','urdf','atlas_convex_hull.urdf');
options.fp = load([getDrakePath(), '/examples/Atlas/data/atlas_fp.mat']);

% Two alternative nominal starting configurations to be used for the other trees:
qNominalC = [-0.047; 0.000; 0.844; 0.000; 0.000; 0.000; 0.434; 0.000; 0.000; 0.270; -0.000; 0.055; -0.604; 1.138; -0.525; -0.055; -1.330; 2.153; 0.500; 0.098; 0.000; 0.001; -0.785; 0.000; -0.055; -0.604; 1.138; -0.525; 0.055; 1.571; 2.532; -1.687; 1.794; 0.426; -3.141; 0.256];
qNominalD = [-0.026; 0.000; 0.844; 0.000; 0.000; 0.000; 0.000; 0.000; 0.000; 0.270; 0.000; 0.055; -0.570; 1.130; -0.550; -0.055; -1.330; 2.153; 0.500; 0.098; 0.000; 0.001; -0.270; 0.000; -0.055; -0.570; 1.130; -0.550; 0.055; 1.330; 2.153; -1.845; 1.353; 0.394; -3.046; 0.256];

r = RigidBodyManipulator(urdf,options);
q_nom = options.fp.xstar(1:r.getNumPositions());

% Create Scene:
world = r.findLinkId('world');
table = RigidBodyBox([.6 1 .025], [.7 0 .9], [0 0 0]);
r = addGeometryToBody(r, world, table);
targetObject = RigidBodyBox([.05 .05 .3], [0.8 0 1.0625], [0 0 0]);
targetObject = targetObject.setColor([1 0 0]);
r = addGeometryToBody(r, world, targetObject);
r = r.compile();


l_foot = r.findLinkId('l_foot');
r_foot = r.findLinkId('r_foot');
g_hand = r.findLinkId('r_hand');
options.point_in_link_frame = [0; -0.3; 0];
% had to move target back from obstacle:
options.targetObjectPos = [0.7 0 1.0625];
%options.targetObjectPos = [0.8 0 1.0625];


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

%Fixed feet constraints
state = options.fp.xstar(1:r.getNumPositions());
kinsol = r.doKinematics(state);

footPose = r.forwardKin(kinsol,l_foot, [0; 0; 0], 2);
leftFootPosConstraint = WorldPositionConstraint(r, l_foot, [0; 0; 0], footPose(1:3), footPose(1:3));
leftFootQuatConstraint = WorldQuatConstraint(r, l_foot, footPose(4:7), 0.0, [0.0, 1.0]);

footPose = r.forwardKin(kinsol,r_foot, [0; 0; 0], 2);
rightFootPosConstraint = WorldPositionConstraint(r, r_foot, [0; 0; 0], footPose(1:3), footPose(1:3));
rightFootQuatConstraint = WorldQuatConstraint(r, r_foot, footPose(4:7), 0.0, [0.0, 1.0]);

%Quasi tatic constraint
l_foot_pts = r.getBody(l_foot).getTerrainContactPoints();
r_foot_pts = r.getBody(r_foot).getTerrainContactPoints();
quasiStaticConstraint = QuasiStaticConstraint(r, [-inf, inf], 1);
quasiStaticConstraint = quasiStaticConstraint.setShrinkFactor(0.5);
quasiStaticConstraint = quasiStaticConstraint.setActive(true);
quasiStaticConstraint = quasiStaticConstraint.addContact(l_foot, l_foot_pts);
quasiStaticConstraint = quasiStaticConstraint.addContact(r_foot, r_foot_pts);

%non-Grasping Hand Distance Constraint
elbow = r.findLinkId('l_larm');
trunk = r.findLinkId('utorso');
nonGraspingHandConstraint = Point2PointDistanceConstraint(r, elbow, trunk, [0; 0; 0], [0; 0; 0], 0.4, Inf);

startPoseConstraints = {leftFootPosConstraint, leftFootQuatConstraint, rightFootPosConstraint, rightFootQuatConstraint,...
  quasiStaticConstraint, nonGraspingHandConstraint};
[q_start, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, startPoseConstraints{:}, ikoptions);
if options.visualize
  v.draw(0,q_start);
  
end

%Set end pose constraints and compute end configuration
goalFrame = [eye(3) options.targetObjectPos'; 0 0 0 1];
goalEulerConstraint = WorldEulerConstraint(r, g_hand, [-pi; -pi/2;0 ], [pi; -pi/2; 0]);
goalDistConstraint = Point2PointDistanceConstraint(r, g_hand, world, options.point_in_link_frame, goalFrame(1:3, 4), -0.001, 0.001);
goalConstraints = {goalDistConstraint, goalEulerConstraint};
endPoseConstraints = [startPoseConstraints, goalConstraints];

[q_end, info, infeasible_constraint] = inverseKin(r, ik_seed_pose, ik_nominal_pose, endPoseConstraints{:}, ikoptions);
if options.visualize
  v.draw(0,q_end);
end

%Create RRTs
inactive_collision_bodies = [l_foot,r_foot];

kinsol = r.doKinematics(q_start);
xyz_quat_start = r.forwardKin(kinsol,g_hand,options.point_in_link_frame,2);
kinsol = r.doKinematics(q_end);
xyz_quat_goal = r.forwardKin(kinsol,g_hand,options.point_in_link_frame,2);
x_start = [xyz_quat_start;q_start];
x_goal = [xyz_quat_goal;q_end];


kinsol = r.doKinematics(qNominalC);
EEpose = r.forwardKin(kinsol, g_hand,  [0; -0.3; 0], 2);
constraints = [startPoseConstraints, generateEEConstraints(r, options, EEpose)];
[qStartC, info, infeasible_constraint] = inverseKin(r, qNominalC, qNominalC, constraints{:}, ikoptions);
if options.visualize
  v.draw(0,qStartC);
end
kinsol = r.doKinematics(qStartC);
xyz_quat_start = r.forwardKin(kinsol,g_hand,options.point_in_link_frame,2);
xStartC = [xyz_quat_start; qStartC];

kinsol = r.doKinematics(qNominalD);
EEpose = r.forwardKin(kinsol, g_hand,  [0; -0.3; 0], 2);
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

finalPose = FinalPoseProblem(r, g_hand, x_start, x_goal, ...
  startPoseConstraints, goalConstraints, q_nom, ...
  'capabilityMap', [], 'graspinghand', 'right', ...
  'activecollisionoptions', ...
  struct('body_idx', setdiff(1:r.getNumBodies(), inactive_collision_bodies)),...
  'ikoptions', ikoptions, ...
  'endeffectorpoint', options.point_in_link_frame);

optionsPlanner = struct();
if ~isfield(optionsPlanner,'costType'), optionsPlanner.costType = 'length'; end;


multiTree = MultipleTreeProblem(r, g_hand, x_start, x_goal, ...
  [xStartC, xStartD], startPoseConstraints, q_nom,...
  'activecollisionoptions', ...
  struct('body_idx', setdiff(1:r.getNumBodies(), inactive_collision_bodies)),...
  'ikoptions', ikoptions, 'endeffectorpoint', options.point_in_link_frame);

[xGoalFull, info] = finalPose.findFinalPose(optionsPlanner);
[multiTree, info, cost, q_path] = multiTree.rrtStar(optionsPlanner, xGoalFull);

if info == 1
  path_length = size(q_path,2);
  xtraj = PPTrajectory(pchip(linspace(0, 1, path_length), [q_path(8:end,:); zeros(r.getNumVelocities(), size(q_path,2))] ));
else
  xtraj = [];
  info = 13;
end


%xtraj = xtraj.setOutputFrame(r.getStateFrame());

rrt_time = toc(rrt_timer);
fprintf(['TIMING:\n',...
  '\ttotal Time: %.2f\n'],...
  rrt_time);

end

function constraints = generateEEConstraints(robot, options, x)
constraints = {};
xyz = x(1:3);
quat = x(4:7);
hand = robot.findLinkId('r_hand');
constraints{1} = WorldPositionConstraint(robot, hand, [0; -0.3; 0], xyz, xyz);
constraints{2} = WorldQuatConstraint(robot, hand, quat, 10*pi/180);
end