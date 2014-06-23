function testForwardKinV

testAtlas();

end

function testAtlas()

% test only works for rpy parameterized robots, since in this case qdot = v
robot = createAtlas('rpy');

testJacobian(robot, 0);
testJacobian(robot, 1);
testJacobian(robot, 2);

testJacobianGradient(robot, 0);
testJacobianGradient(robot, 1);
testJacobianGradient(robot, 2);

end

function testJacobian(robot, rotation_type)

nv = robot.getNumVelocities();
nb = length(robot.body);
body_range = [1, nb];

n_tests = 10;
test_number = 1;

while test_number < n_tests
  q = getRandomConfiguration(robot);
  v = randn(nv, 1);
  kinsol = robot.doKinematics(q, false, false, v, true);
  
  base = randi(body_range);
  end_effector = randi(body_range);
  if base ~= end_effector
    nPoints = randi([1, 10]);
    points = randn(3, nPoints);
    
    [~, J] = robot.forwardKinV(kinsol, end_effector, points, rotation_type, base);
    
    option.grad_method = 'taylorvar';
    [~, J_geval] = geval(1, @(q) robot.forwardKinV(robot.doKinematics(q, false, false), end_effector, points, rotation_type, base), q, option);
    valuecheck(J_geval, J, 1e-10);
   
    test_number = test_number + 1;
  end
end

end

function testJacobianGradient(robot, rotation_type)
nv = robot.getNumVelocities();
nb = length(robot.body);
body_range = [1, nb];

n_tests = 5;
test_number = 1;
while test_number < n_tests
  base = randi(body_range);
  end_effector = randi(body_range);

  if base ~= end_effector
    q = getRandomConfiguration(robot);
    v = randn(nv, 1);
    nPoints = randi([1, 10]);
    points = randn(3, nPoints);
    
    option.grad_method = 'taylorvar';
    kinsol = robot.doKinematics(q, true, false);
    [~, ~, dJ] = robot.forwardKinV(kinsol, end_effector, points, rotation_type, base);
    [~, dJ_geval] = geval(1, @(q) gevalFunction(robot, q, v, end_effector, points, rotation_type, base), q, option);
    valuecheck(dJ_geval, dJ, 1e-10);
    test_number = test_number + 1;
  end
end

end

function J = gevalFunction(robot, q, v, end_effector, points, rotation_type, base)
kinsol = robot.doKinematics(q, false, false, v, true);
[~, J] = robot.forwardKinV(kinsol, end_effector, points, rotation_type, base);
end