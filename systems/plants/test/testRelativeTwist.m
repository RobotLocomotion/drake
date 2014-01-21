function testRelativeTwist()
testAtlas('rpy');
end

%% test versus geometric jacobian * joint velocities
function testAtlas(floatingJointType)
robot = createAtlas(floatingJointType);

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
  base = randi(bodyRange);
  endEffector = randi(bodyRange);
  expressedIn = randi(bodyRange);
  
  if base ~= endEffector
    twist = robot.relativeTwist(kinsol, base, endEffector, expressedIn);
    [jacobian, vIndices] = robot.geometricJacobian(kinsol, base, endEffector, expressedIn);
    twistViaJacobian = jacobian * (v(vIndices));
    valuecheck(twistViaJacobian, twist, 1e-12);
    testNumber = testNumber + 1;
  end
end
end