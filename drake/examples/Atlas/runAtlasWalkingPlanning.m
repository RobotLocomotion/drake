function xtraj = runAtlasWalkingPlanning(options)
% Demonstration of footstep and walking planning on Atlas. First, the footstep planning
% step generates a set of reachable footstep poses. Next, we use those foot poses to
% plan a trajectory of the Zero Moment Point (ZMP), from which we can derive a
% trajectory for the robot's Center of Mass. From this, we can plan a full state
% trajectory for the robot.
% @retval plan a WalkingPlanData object including the zmp and com trajectories

if nargin<1, options = struct(); end
if ~isfield(options,'terrain'), options.terrain = RigidBodyFlatTerrain(); end
if ~isfield(options,'navgoal'), options.navgoal = [1.5;0;0;0;0;pi/2]; end
if ~isfield(options,'safe_regions'), options.safe_regions = []; end
if ~isfield(options,'step_params'), options.step_params = struct(); end

checkDependency('lcmgl');

% Set up the model
r = Atlas('urdf/atlas_minimal_contact.urdf',options);
v = r.constructVisualizer();

if ~isfield(options, 'x0')
  load('data/atlas_fp.mat', 'xstar');
  x0 = xstar;
  if isfield(options,'initial_pose'), x0(1:6) = options.initial_pose; end
  x0 = r.resolveConstraints(x0);
else
  x0 = options.x0;
  options.initial_state = x0(1:6);
end

r = r.setInitialState(x0);
v.draw(0, x0);

q0 = x0(1:r.getNumPositions());

R=rotz(options.navgoal(6));
rfoot_navgoal = options.navgoal;
lfoot_navgoal = options.navgoal;
rfoot_navgoal(1:3) = rfoot_navgoal(1:3) + R*[0;-0.13;0];
lfoot_navgoal(1:3) = lfoot_navgoal(1:3) + R*[0;0.13;0];

% Plan footsteps to the goal
goal_pos = struct('right', rfoot_navgoal, 'left', lfoot_navgoal);
footstep_plan = r.planFootsteps(q0, goal_pos, options.safe_regions, struct('method_handle', @footstepPlanner.humanoids2014, 'step_params', options.step_params));
if length(footstep_plan.footsteps) < 3
  error('Drake:NoFeasibleFootstepPlan', 'No feasible footstep plan could be found');
end

% Snap to terrain
nsteps = length(footstep_plan.footsteps);
for j = 3:nsteps
  if ~footstep_plan.footsteps(j).pos_fixed(3)
    footstep_plan.footsteps(j) = fitStepToTerrain(r, footstep_plan.footsteps(j));
  end
end


% Add terrain profiles
for j = 3:nsteps
  [~, contact_width] = contactVolume(r, ...
                                        footstep_plan.footsteps(j-2), ...
                                        footstep_plan.footsteps(j));
  footstep_plan.footsteps(j).terrain_pts = sampleSwingTerrain(r, footstep_plan.footsteps(j-2), footstep_plan.footsteps(j), contact_width/2, struct());
end


% Show the result
walking_plan_data = r.planWalkingZMP(x0, footstep_plan);
[xtraj, htraj, ts] = r.planWalkingStateTraj(walking_plan_data.settings);

if isa(v, 'BotVisualizer')
  lcmgl = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'footstep_plan');
  footstep_plan.draw_lcmgl(lcmgl);
  lcmgl.switchBuffers();
  lcmgl = drake.matlab.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'walking_plan');
  walking_plan_data.settings.draw_lcmgl(lcmgl);
  lcmgl.switchBuffers();
else
  figure(25)
  footstep_plan.draw_2d();
end

v.playback(xtraj, struct('slider', true));


end

