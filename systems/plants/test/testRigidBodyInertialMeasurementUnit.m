function testRigidBodyInertialMeasurementUnit()

testVersusTwistMethod(createAtlas('rpy'));
compare2dTo3d();

end

function testVersusTwistMethod(r)

nbodies = r.getNumBodies();
body = randi([2 nbodies]);
xyz = randn(3, 1);
rpy = randn(3, 1); 
[r,imu_frame] = addFrame(r,RigidBodyFrame(body,xyz,rpy,'imu'));
r = r.addSensor(RigidBodyInertialMeasurementUnit(r, imu_frame));
r = compile(r);

nq = r.getNumPositions();
nv = r.getNumVelocities();

for i = 1 : 100
  t = rand();
  q = randn(nq, 1); % TODO: use getRandomConfiguration once floatingBase branch is merged in so that it also works for quat parameterized robots
  v = randn(nv, 1);
  x = [q; v];
  u = randn(r.getNumInputs(), 1);
  y = r.output(t, x, u);
  
  quat = y(1:4);
  omega = y(5:7);
  accel = y(8:10);
  
  kinsol = r.doKinematics(q, false, false, v);
  Tbody = kinsol.T{body};
  Rframe = Tbody(1:3, 1:3) * rpy2rotmat(rpy);
  quat_check = rotmat2quat(Rframe);
  quat_difference = quatDiff(quat, quat_check);
  qs = quat_difference(1);
  qvec = quat_difference(2:4);
  angle_difference = angleDiff(0, 2 * atan2(norm(qvec), qs));
  valuecheck(0, angle_difference, 1e-10);
  
  world = 1;
  twists = r.twists(kinsol.T, q, v);
  body_twist = relativeTwist(kinsol.T, twists, world, body, world);
  omega_check = Rframe' * body_twist(1:3);  % TODO: change to use relativeTwist with frame once floatingBase branch is merged in
  valuecheck(omega_check, omega, 1e-10);

  xdot = r.dynamics(t, x, u);
  vd = xdot(nq + 1 : end);

  spatial_accels = r.spatialAccelerations(kinsol.T, twists, q, v, vd);
  body_spatial_accel = transformSpatialAcceleration(spatial_accels{body}, kinsol.T, twists, world, body, body, world); % TODO: update once floatingBase branch is merged in: output of spatialAccelerations changed from body frame to world frame
  
  % TODO: extract this out into a method of its own:
  p = r.forwardKin(kinsol, body, xyz, 0);
  omega_body = body_twist(1:3);
  v_body = body_twist(4:6);
  omegad_body = body_spatial_accel(1:3);
  vd_body = body_spatial_accel(4:6);
  accel_check_world = cross(omegad_body, p) + vd_body + cross(omega_body, cross(omega_body, p) + v_body);
  accel_check_body = Rframe' * accel_check_world;
  valuecheck(accel_check_body, accel, 1e-10);
end
end

function compare2dTo3d()
rng(55, 'twister'); % angledot wrong on first test
% rng(541, 'twister'); % angledot wrong on first test
% rng(125, 'twister'); % angledot right on first couple of tests, but accels wrong due to rotation in wrong direction in RigidBodyInertialMeasurementUnit
r2d = PlanarRigidBodyManipulator('DoublePendWBiceptSpring.urdf');
r3d = RigidBodyManipulator('DoublePendWBiceptSpring.urdf');

% works:
% options.view = 'front';
% r2d = PlanarRigidBodyManipulator('DoublePendWBiceptSpringFront.urdf', options);
% r3d = RigidBodyManipulator('DoublePendWBiceptSpringFront.urdf');
assert(getNumStates(r2d)==getNumStates(r3d));

nbodies = r2d.getNumBodies();
body2d = randi([2 nbodies]);
b = getBody(r2d,body2d);
body3d = findLinkInd(r3d,b.linkname);
rpy = randn*r2d.view_axis; % randn(3, 1);
xyz = randn(3, 1);

[r2d,imu_frame] = addFrame(r2d,RigidBodyFrame(body2d,xyz,rpy,'imu'));
r2d = r2d.addSensor(RigidBodyInertialMeasurementUnit(r2d, imu_frame));
r2d = compile(r2d);

[r3d,imu_frame] = addFrame(r3d,RigidBodyFrame(body3d,xyz,rpy,'imu'));
r3d = r3d.addSensor(RigidBodyInertialMeasurementUnit(r3d, imu_frame));
r3d = compile(r3d);

nq = r2d.getNumPositions();
nv = r2d.getNumVelocities();
nu = r2d.getNumInputs();
for i = 1 : 100
  t = rand();
  q = randn(nq, 1); % TODO: use getRandomConfiguration once floatingBase branch is merged in so that it also works for quat parameterized robots
  v = randn(nv, 1);
  x = [q; v];
  u = randn(nu, 1);
  y2d = r2d.output(t, x, u);
  y3d = r3d.output(t, x, u);
  
  angle = y2d(1);
  angledot = y2d(2);
  accel2d = y2d(3:4);
  
  quat = y3d(1:4);
  omega = y3d(5:7);
  accel = y3d(8:10);
  axis = quat2axis(quat);
  if dot(axis(1:3),r2d.view_axis)<0
    axis = -axis;
    omega = -omega;
  end
  valuecheck(axis(1:3),r2d.view_axis);
  valuecheck(0, angleDiff(axis(4), angle));
  valuecheck(r2d.view_axis'*omega, angledot);
  valuecheck(r2d.T_2D_to_3D' * accel, accel2d);
end
end
