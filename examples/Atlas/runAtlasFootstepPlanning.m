function plan = runAtlasFootstepPlanning()

% Set up the model
load('data/atlas_fp.mat', 'xstar');
r = Atlas('urdf/atlas_minimal_contact.urdf');
r = r.setInitialState(xstar);

% Find the initial positions of the feet
kinsol = doKinematics(r, xstar(1:r.getNumDOF()));
start_pos = struct('right', forwardKin(r, kinsol, r.foot_frame_id.right, [0;0;0], 1), ...
                   'left', forwardKin(r, kinsol, r.foot_frame_id.left, [0;0;0], 1));
                 
% Plan footsteps to the goal
goal_pos = struct('right', [1;0;0;0;0;0], 'left', [1;0.26;0;0;0;0]);
plan = r.planFootsteps(start_pos, goal_pos);

% Show the result
v = r.constructVisualizer();
v.draw(0, xstar);
if isa(v, 'BotVisualizer')
  checkDependency('lcmgl');
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'footstep_plan');
  plan.draw_lcmgl(lcmgl);
else
  figure(25)
  plan.draw_2d();
end

end

