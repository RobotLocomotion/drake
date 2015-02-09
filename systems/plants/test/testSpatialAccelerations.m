function testSpatialAccelerations()
testAtlas();
end

function testAtlas()
robot = createAtlas('rpy');
testVersusNumericalDifferentiation(robot);
end

function testVersusNumericalDifferentiation(robot)
dt = 1e-8;
nq = robot.getNumPositions();
nv = robot.getNumVelocities();

nBodies = length(robot.body);

nTests = 50;
for i = 1 : nTests
  q = randn(nq, 1);
  v = randn(nv, 1);
  options.use_mex = false;
  kinsol = robot.doKinematics(q, v, options);
  twists = kinsol.twists;
  
  vd = randn(nv, 1);
  twistdot = spatialAccelerations(robot, kinsol, vd);

  q = q + v * dt;
  v = v + vd * dt;
  kinsolNew = robot.doKinematics(q, v, options);
  twistsNew = kinsolNew.twists;
  
  for j = 1 : nBodies
    spatialAccelerationNumericalDiff = (twistsNew{j} - twists{j}) / dt;
    valuecheck(twistdot{j}, spatialAccelerationNumericalDiff, 1e-5);
  end
end
end