function testQdotTov
testFallingBrick('quat');
testAtlas('rpy');
testAtlas('quat');
end

function testFallingBrick(floatingType)
options.floating = floatingType;
robot = RigidBodyManipulator('FallingBrick.urdf',options);
testVersusInverse(robot);
testGradient(robot);
end

function testAtlas(floatingType)
robot = createAtlas(floatingType);
testVersusInverse(robot);
testGradient(robot);
end

function testVersusInverse(robot)
nv = robot.getNumVelocities();
q = getRandomConfiguration(robot);
kinsol = doKinematics(robot, q, false, false);
valuecheck(eye(nv), kinsol.qdotToV * kinsol.vToqdot, 1e-12);
end

function testGradient(robot)
q = getRandomConfiguration(robot);
[~, dVq] = robot.qdotToV(q);
option.grad_method = 'taylorvar';
[~, dVq_geval] = geval(1, @robot.qdotToV, q, option);
valuecheck(dVq_geval, dVq, 1e-10);

[~, dVqInv] = robot.vToqdot(q);
option.grad_method = 'taylorvar';
[~, dVqInv_geval] = geval(1, @robot.vToqdot, q, option);
valuecheck(dVqInv_geval, dVqInv, 1e-10);
end
