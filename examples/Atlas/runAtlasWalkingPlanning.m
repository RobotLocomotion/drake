function plan = runAtlasWalkingPlanning()

% Set up the model
load('data/atlas_fp.mat', 'xstar');
x0 = xstar;
r = Atlas('urdf/atlas_minimal_contact.urdf');
r = r.setInitialState(x0);

% Find the initial positions of the feet
kinsol = doKinematics(r, x0(1:r.getNumDOF()));
start_pos = struct('right', forwardKin(r, kinsol, r.foot_frame_id.right, [0;0;0], 1), ...
                   'left', forwardKin(r, kinsol, r.foot_frame_id.left, [0;0;0], 1));

% Plan footsteps to the goal
goal_pos = struct('right', [1;0;0;0;0;0], 'left', [1;0.26;0;0;0;0]);
footstep_plan = r.planFootsteps(start_pos, goal_pos);

% Show the result
v = r.constructVisualizer();
v.draw(0, x0);
walking_plan_data = r.planWalkingZMP(x0, footstep_plan);
[xtraj, htraj, ts] = r.planWalkingStateTraj(walking_plan_data);

if isa(v, 'BotVisualizer')
  checkDependency('lcmgl');
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'footstep_plan');
  footstep_plan.draw_lcmgl(lcmgl);
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'walking_plan');
  walking_plan_data.draw_lcmgl(lcmgl);
else
  figure(25)
  footstep_plan.draw_2d();
end

v.playback(xtraj);



end

