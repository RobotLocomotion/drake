options.terrain = RigidBodyFlatTerrain();
options.floating = true;
options.ignore_self_collisions = true;
options.use_bullet = false;
options.use_new_kinsol = true;
p = RigidBodyManipulator('OneLegHopper.urdf',options);
r = TimeSteppingRigidBodyManipulator(p,.001);
v = p.constructVisualizer();
load hopper_lqr
for i=1:length(c)
  c{i} = c{i}.setOutputFrame(r.getInputFrame);
  c{i} = c{i}.setInputFrame(r.getOutputFrame);
  sys_cl{i} = r.feedback(c{i});
end

x0 = xtraj{1}.eval(0);

%%
for i=1:2,
  tspan = xtraj{i}.tspan;
  tspan(1) = floor(tspan(1)/r.timestep)*r.timestep;
  tspan(2) = floor(tspan(2)/r.timestep)*r.timestep;
  traj_cl{i} = sys_cl{i}.simulate(tspan,x0);
  x0 = traj_cl{i}.eval(traj_cl{i}.tspan(2));
  v.playback(traj_cl{i});
end