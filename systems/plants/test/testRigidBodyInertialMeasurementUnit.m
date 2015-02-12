function testRigidBodyInertialMeasurementUnit()

testAtlas('rpy');
compare2dTo3d();

end

function testAtlas(floatingtype)
r = createAtlas(floatingtype);

nbodies = r.getNumBodies();
body = randi([2 nbodies]);
xyz = randn(3, 1);
rpy = zeros(3, 1); % TODO: change once nonzero rpy is supported
imu = RigidBodyInertialMeasurementUnit(r, body, xyz, rpy);
r = r.addSensor(imu);
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
  kinsol.twists = r.twists(kinsol.T, q, v);
  kinsol.v = v;
  Tbody = kinsol.T{body};
  Rframe = Tbody(1:3, 1:3) * rpy2rotmat(rpy);
  quat_check = rotmat2quat(Rframe);
  quat_difference = quatDiff(quat, quat_check);
  qs = quat_difference(1);
  qvec = quat_difference(2:4);
  angle_difference = angleDiff(0, 2 * atan2(norm(qvec), qs));
  valuecheck(0, angle_difference, 1e-10);
  
  world = 1;
  body_twist_in_world = relativeTwist(kinsol, world, body, world);
  omega_check = Rframe' * body_twist_in_world(1:3);  % TODO: change to use relativeTwist with frame once floatingBase branch is merged in
  valuecheck(omega_check, omega, 1e-10);

  xdot = r.dynamics(t, x, u);
  vd = xdot(nq + 1 : end);
  spatial_accels = r.spatialAccelerations(kinsol, vd);
  body_spatial_accel_in_world = spatial_accels{body};
  
  % TODO: extract this out into a method of its own:
  p = r.forwardKin(kinsol, body, xyz, 0);
  omega_body = body_twist_in_world(1:3);
  v_body = body_twist_in_world(4:6);
  omegad_body = body_spatial_accel_in_world(1:3);
  vd_body = body_spatial_accel_in_world(4:6);
  accel_check_world = cross(omegad_body, p) + vd_body + cross(omega_body, cross(omega_body, p) + v_body);
  accel_check_body = Rframe' * accel_check_world;
  valuecheck(accel_check_body, accel, 1e-10);
end
end

function compare2dTo3d()
r2d = PlanarRigidBodyManipulator('DoublePendWBiceptSpring.urdf');
r3d = RigidBodyManipulator('DoublePendWBiceptSpring.urdf');

nbodies = r2d.getNumBodies();
body = nbodies;
rpy = zeros(3, 1); % TODO: change once nonzero rpy is supported
xyz = randn(3, 1);
xy = xyz(1:2);

imu2d = RigidBodyInertialMeasurementUnit(r2d, body, xy, rpy);
r2d = r2d.addSensor(imu2d);
r2d = compile(r2d);

imu3d = RigidBodyInertialMeasurementUnit(r3d, body, xyz, rpy);
r3d = r3d.addSensor(imu3d);
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
  
  valuecheck(0, angleDiff(axis(4), angle));
  valuecheck(omega(3), angledot);
  valuecheck(accel(1:2), accel2d);
end
end
