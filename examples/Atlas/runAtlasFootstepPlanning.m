function plan = runAtlasFootstepPlanning()

load('data/atlas_fp.mat', 'xstar');
r = Atlas('urdf/atlas_minimal_contact.urdf');
r = r.setInitialState(xstar);
r = compile(r);
l_id = r.findFrameId('l_foot_sole');
r_id = r.findFrameId('r_foot_sole');
kinsol = doKinematics(r, xstar(1:r.getNumDOF()));

start_pos = struct('right', forwardKin(r, kinsol, r_id, [0;0;0], 1), 'left', forwardKin(r, kinsol, l_id, [0;0;0], 1));
goal_pos = struct('right', [1;0;0;0;0;0], 'left', [1;0.26;0;0;0;0]);
plan = r.planFootsteps(start_pos, goal_pos);

v = r.constructVisualizer();
v.draw(0, xstar);
if isa(v, 'BotVisualizer')
  lcmgl = drake.util.BotLCMGLClient(lcm.lcm.LCM.getSingleton(), 'footstep_plan');
  plan.draw_lcmgl(lcmgl);
else
  figure(25)
  plan.draw_2d();
end

end

