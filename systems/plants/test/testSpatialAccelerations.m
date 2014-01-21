function testSpatialAccelerations()
testAtlas();
end

function testAtlas()
robot = createAtlas(''); % TODO: need to test 6-DoF joints, but roll/pitch/yaw parameterization has time-variant motion subspace
testVersusNumericalDifferentiation(robot);
end

function testVersusNumericalDifferentiation(robot)
dt = 1e-8;
nq = robot.getNumStates() / 2; % TODO
nv = robot.getNumStates() / 2; % TODO

nBodies = length(robot.body);

nTests = 50;
for i = 1 : nTests
  q = randn(nq, 1);
  v = randn(nv, 1);
  kinsol = robot.doKinematics(q, false, false, v);
  
  vd = randn(nv, 1);
  spatialAccelerations = robot.spatialAccelerations(kinsol, q, v, vd);

  q = q + v * dt;
  v = v + vd * dt;
  kinsolNew = robot.doKinematics(q, false, false, v);
  
  for j = 1 : nBodies
    spatialAccelerationNumericalDiff = (kinsolNew.twist{j} - kinsol.twist{j}) / dt;
    valuecheck(spatialAccelerations{j}, spatialAccelerationNumericalDiff, 1e-5);
  end
end
end