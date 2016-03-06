function xtraj = testRigidBodyBluffBody

  % Tests control surfaces by creating from a URDF that has control surfaces
  % and simulating.

  %% Construct from URDF


  disp('Constructing from URDF...');

  options.floating = true;
  r = RigidBodyManipulator('testRigidBodyBluffBody.urdf', options);


  params = r.getParams();
  params.drag_area = 0; % no drag force for the first simulation
  r = r.setParams(params);
  
  v = r.constructVisualizer();

  %% simulate

  disp('Simulating without drag force...');

  x0 = [0; 0; 0; 0; 0; 0; 15; 0; 0; 0; 0; 0];
  end_t = .5;

  constant_traj = ConstantTrajectory([0.8 0.8 100]);
  constant_traj = constant_traj.setOutputFrame(r.getInputFrame());

  feedback_system = cascade(constant_traj, r);

  [ytraj, xtraj] = feedback_system.simulate([0 end_t], x0);

  v.playback(xtraj)

  final_x = xtraj.eval(end_t)

  %% build with drag force

  % now simulate with drag forces
  disp('Simulating with drag forces...');
  
  params = r.getParams();
  params.drag_area = 0.01; % no drag force for the first simulation
  r2 = r.setParams(params);

  v = r2.constructVisualizer();

  constant_traj = ConstantTrajectory([0.8 0.8 100]);
  constant_traj = constant_traj.setOutputFrame(r2.getInputFrame());

  feedback_system = cascade(constant_traj, r2);

  %% simulate with drag force

  [ytraj, xtraj] = feedback_system.simulate([0 end_t], x0);

  v.playback(xtraj)


  final_x_with_drag = xtraj.eval(end_t)

  assert(final_x_with_drag(1) < final_x(1)) % should move forward less with drag
  assert(abs(final_x_with_drag(2)) < .1) % should stay in the center regardless of drag
  assert(final_x_with_drag(3) < final_x(3)); % should gain less altitude with drag

end