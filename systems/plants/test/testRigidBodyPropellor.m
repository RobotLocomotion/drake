function xtraj = testRigidBodyPropellor
  % Tests propellors

  %% Construct from URDF


  disp('Constructing from URDF...');

  options.floating = true;
  r = RigidBodyManipulator('testRigidBodyPropellor.urdf', options);

  v = r.constructVisualizer();


  %% simulate with the front propellor running

  disp('Simulating with forwards facing prop running...');

  x0 = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
  end_t = .5;

  constant_traj = ConstantTrajectory([100 0]);
  constant_traj = constant_traj.setOutputFrame(r.getInputFrame());

  feedback_system = cascade(constant_traj, r);

  [ytraj, xtraj] = feedback_system.simulate([0 end_t], x0);

  v.playback(xtraj)

  final_x = xtraj.eval(end_t)
  
  assert(final_x(1) > 1) % should move forward
  assert(final_x(3) < 1) % should fall due to gravity
  assert(abs(final_x(2)) < 0.1) % shouldn't move much in Y
  assert(final_x(4) > 1) % should have velocity in the x direction
  
  assert(final_x(10) > 5) % should have lots of positive roll

  %% simulate with the upwards propellor running

  disp('Simulating with upwards facing prop runnings...');

  constant_traj = ConstantTrajectory([0 100]);
  constant_traj = constant_traj.setOutputFrame(r.getInputFrame());

  feedback_system = cascade(constant_traj, r);

  %% simulate with drag force

  [ytraj, xtraj] = feedback_system.simulate([0 end_t], x0);

  v.playback(xtraj)


  final_x = xtraj.eval(end_t)

  assert(abs(final_x(1)) < .1) % shouldn't move forward much
  assert(final_x(3) > 1) % should move up
  assert(abs(final_x(2)) < 0.1) % shouldn't move much in Y
  assert(final_x(6) > 1) % should have velocity in the z direction
  
  assert(final_x(12) > 5) % should have lots of positive yaw
  
end