function testAnalyticalJacobian

testAtlas('rpy');
% testAtlas('quat'); % TODO: generating robot currently results in an error.

end

function testAtlas(floatingJointType)

robot = createAtlas(floatingJointType);

compareToForwardKin(robot, 0);
compareToForwardKin(robot, 1);
compareToForwardKin(robot, 2);

end

function compareToForwardKin(robot, rotationType)

nv = robot.getNumVelocities();
nBodies = length(robot.body);
bodyRange = [1, nBodies];

analyticalJacobianTime = 0;
forwardKinTime = 0;

nTests = 200;
testNumber = 1;
while testNumber < nTests
  q = getRandomConfiguration(robot);
  v = randn(nv, 1);
  kinsol = robot.doKinematics(q,false,false, v);

  base = 1;
  endEffector = randi(bodyRange);
  if base ~= endEffector
    nPoints = randi([1, 10]);
    points = randn(3, nPoints);

    tic
    J = robot.analyticalJacobian(kinsol, base, endEffector, points, rotationType);
    analyticalJacobianTime = analyticalJacobianTime + toc * 1e3;

    tic
    [~, JForwardKin] = robot.forwardKin(kinsol, endEffector, points, rotationType);
    forwardKinTime = forwardKinTime + toc * 1e3;

    valuecheck(J, JForwardKin, 1e-8);
    testNumber = testNumber + 1;
  end
end
analyticalJacobianTime = analyticalJacobianTime / nTests;
forwardKinTime = forwardKinTime / nTests;

displayComputationTime = true;
if displayComputationTime
  fprintf('analyticalJacobianTime: %0.3f\n', analyticalJacobianTime);
  fprintf('forwardKinTime: %0.3f\n', forwardKinTime);
  fprintf('\n');
end
end

% TIMEOUT 1500
