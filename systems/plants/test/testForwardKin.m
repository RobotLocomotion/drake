function testForwardKin

testAtlas('rpy');
% testAtlas('quat'); % TODO: generating robot currently results in an error.

end

function testAtlas(floatingJointType)

robot = createAtlas(floatingJointType);

compareToNumerical(robot, 0);
compareToNumerical(robot, 1);
compareToNumerical(robot, 2);

end

function compareToNumerical(robot, rotationType)

nq = robot.getNumPositions();
nv = robot.getNumVelocities();

nBodies = length(robot.body);

bodyRange = [1, nBodies];

computationTime = 0;

nTests = 10;
testNumber = 1;
delta = 1e-10;
epsilon = 1e-3;
while testNumber < nTests
  q = randn(nq, 1);
  v = randn(nv, 1);
  kinsol = robot.doKinematics(q, true, false, v);
  
  base = 1;
  endEffector = randi(bodyRange);
  if base ~= endEffector
    nPoints = randi([1, 10]);
    points = randn(3, nPoints);
    
    tic
    [x, J, JdotV] = robot.forwardKin(kinsol, endEffector, points, rotationType, base);
    computationTime = computationTime + toc * 1e3;
    
    for col = 1 : length(q)
      qDelta = q;
      qDelta(col) = qDelta(col) + delta;
      kinsol = robot.doKinematics(qDelta, false, false);
      xDelta = robot.forwardKin(kinsol, endEffector, points, rotationType, base);
      dxdqNumerical = (xDelta - x) / delta;
      valuecheck(J(:, col), dxdqNumerical(:), epsilon);
    end
    
    qDelta = q + delta * v;
    kinsol = robot.doKinematics(qDelta, false, false);
    [~, JDelta] = robot.forwardKin(kinsol, endEffector, points, rotationType, base);
    JdotNumerical = (JDelta - J) / delta;
    valuecheck(JdotV, JdotNumerical * v, epsilon);
    
    testNumber = testNumber + 1;
  end
end

displayComputationTime = false;
if displayComputationTime
  fprintf('computation time per call: %0.3f ms\n', computationTime / nTests);
  fprintf('\n');
end
end