function xtraj = testRigidBodyWingChangingParams()

  % Tests control surfaces by creating from a URDF that has control surfaces
  % and simulating.

  %% Construct from URDF


  disp('Constructing from URDF...');

  options.floating = true;
  r = RigidBodyManipulator('testRigidBodyWingChangingParams.urdf', options);
  v = r.constructVisualizer();

  %% simulate

  disp('Simulating...');

  x0 = [0; 0; 0; 0; 0; 0; 15; 0; 0; 0; 0; 0];
  end_t = .5;

  constant_traj = ConstantTrajectory([0.8 0.8 100]);
  constant_traj = constant_traj.setOutputFrame(r.getInputFrame());

  feedback_system = cascade(constant_traj, r);

  [ytraj, xtraj] = feedback_system.simulate([0 end_t], x0);

  v.playback(xtraj)

  final_x = xtraj.eval(end_t);


  assert(final_x(1) > 5) % should move forward
  assert(abs(final_x(2)) < .1) % should stay in the center
  assert(final_x(3) > 1); % should gain altitude
  
  
  
  %% change the params
  
  params = r.getParams();
  params.cs_chord = params.cs_chord + .1;
  params.cs_span = params.cs_span - .1;
  r2 = r.setParams(params);
  
  v2 = r2.constructVisualizer();
  
  constant_traj = ConstantTrajectory([0.8 0.8 100]);
  constant_traj = constant_traj.setOutputFrame(r2.getInputFrame());

  feedback_system = cascade(constant_traj, r2);

  [ytraj, xtraj] = feedback_system.simulate([0 end_t], x0);

  v2.playback(xtraj)

  final_x_different_params = xtraj.eval(end_t);


  assert(final_x(1) > 5) % should move forward
  assert(abs(final_x(2)) < .1) % should stay in the center
  assert(final_x(3) > 1); % should gain altitude

end