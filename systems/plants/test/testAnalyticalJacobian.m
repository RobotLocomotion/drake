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

nq = robot.getNumStates() / 2; % TODO
nv = robot.getNumStates() / 2; % TODO

nBodies = length(robot.body);

bodyRange = [1, nBodies];

nTests = 50;
testNumber = 1;
while testNumber < nTests
  q = randn(nq, 1);
  v = randn(nv, 1);
  kinsol = robot.doKinematics(q,false,false, v);
  
  base = 1;
  endEffector = randi(bodyRange);
  if base ~= endEffector
    nPoints = randi([1, 10]);
    points = randn(3, nPoints);
    
    J = robot.analyticalJacobian(kinsol, base, endEffector, points, rotationType);
    [~, JForwardKin] = robot.forwardKin(kinsol, endEffector, points, rotationType);
    
    valuecheck(J, JForwardKin, 1e-8);
    testNumber = testNumber + 1;
  end
end
end