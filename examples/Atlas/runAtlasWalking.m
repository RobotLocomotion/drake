function runAtlasWalking(example_options)
% Example running walking QP controller from
% Scott Kuindersma, Frank Permenter, and Russ Tedrake.
% An efficiently solvable quadratic program for stabilizing dynamic
% locomotion. In Proceedings of the International Conference on 
% Robotics and Automation, Hong Kong, China, May 2014. IEEE.
%
% @option use_mex
% @option use_bullet
% @option use_angular_momentum
% @options navgoal
% 

checkDependency('gurobi');

if nargin<1, example_options=struct(); end
if ~isfield(example_options,'use_mex'), example_options.use_mex = true; end
if ~isfield(example_options,'use_bullet') example_options.use_bullet = false; end
if ~isfield(example_options,'use_angular_momentum') example_options.use_angular_momentum = false; end
if ~isfield(example_options,'navgoal')
%  navgoal = [2*rand();0.25*randn();0;0;0;0];
  example_options.navgoal = [1.5;0;0;0;0;0];
end
if ~isfield(example_options,'terrain'), example_options.terrain = RigidBodyFlatTerrain; end

% silence some warnings
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints')
warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits')

% construct robot model
options.floating = true;
options.ignore_self_collisions = true;
options.ignore_friction = true;
options.dt = 0.001;
options.terrain = example_options.terrain;
r = Atlas(fullfile(getDrakePath,'examples','Atlas','urdf','atlas_minimal_contact.urdf'),options);
r = r.removeCollisionGroupsExcept({'heel','toe'});
r = compile(r);

% set initial state to fixed point
load(fullfile(getDrakePath,'examples','Atlas','data','atlas_fp.mat'));
if isfield(options,'initial_pose'), xstar(1:6) = options.initial_pose; end
xstar = r.resolveConstraints(xstar);
r = r.setInitialState(xstar);

v = r.constructVisualizer;
v.display_dt = 0.03;

nq = getNumPositions(r);

x0 = xstar;

% Find the initial positions of the feet
R=rotz(example_options.navgoal(6));

rfoot_navgoal = example_options.navgoal;
lfoot_navgoal = example_options.navgoal;

rfoot_navgoal(1:3) = rfoot_navgoal(1:3) + R*[0;-0.13;0];
lfoot_navgoal(1:3) = lfoot_navgoal(1:3) + R*[0;0.13;0];

% Plan footsteps to the goal
goal_pos = struct('right', rfoot_navgoal, 'left', lfoot_navgoal);
footstep_plan = r.planFootsteps(x0(1:nq), goal_pos);

walking_plan_data = r.planWalkingZMP(x0(1:r.getNumPositions()), footstep_plan);

traj = atlasUtil.simulateWalking(r, walking_plan_data, example_options.use_mex, false, example_options.use_bullet, example_options.use_angular_momentum, true);

playback(v,traj,struct('slider',true));

[com, rms_com] = atlasUtil.plotWalkingTraj(r, traj, walking_plan_data);

if rms_com > length(footstep_plan.footsteps)*0.5
  error('runAtlasWalking unit test failed: error is too large');
  example_options.navgoal
end

% make sure we're at least vaguely close to the goal
valuecheck(com(1:3,end), [example_options.navgoal(1:2); 0.9], 0.2);

end

% TIMEOUT 1500
