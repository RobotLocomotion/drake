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

nq = robot.getNumPositions();

nBodies = length(robot.body);

bodyRange = [1, nBodies];

computationTime = 0;

nTests = 10;
testNumber = 1;
delta = 1e-10;
epsilon = 1e-3;
while testNumber < nTests
  q = randn(nq, 1);
  kinsol = robot.doKinematics(q, false, false);
  
  base = 1;
  endEffector = randi(bodyRange);
  if base ~= endEffector
    nPoints = randi([1, 10]);
    points = randn(3, nPoints);
    
    tic
    [x, J] = robot.analyticalJacobian(kinsol, base, endEffector, points, rotationType);
    computationTime = computationTime + toc * 1e3;
    
    for col = 1 : length(q)
      qDelta = q;
      qDelta(col) = qDelta(col) + delta;
      kinsol = robot.doKinematics(qDelta, false, false);
      xDelta = robot.analyticalJacobian(kinsol, base, endEffector, points, rotationType);
      dxdq = (xDelta - x) / delta;
      valuecheck(J(:, col), dxdq(:), epsilon);
    end
    
    testNumber = testNumber + 1;
  end
end

displayComputationTime = false;
if displayComputationTime
  fprintf('computation time per call: %0.3f ms\n', computationTime / nTests);
  fprintf('\n');
end
end