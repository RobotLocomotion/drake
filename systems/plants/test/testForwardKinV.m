function testForwardKinV

testAtlas();

end

function testAtlas()

% test only works for rpy parameterized robots, since in this case qdot = v
robot = createAtlas('rpy');

testGradients(robot, 0);
testGradients(robot, 1);
testGradients(robot, 2);

end

function testGradients(robot, rotation_type)
nb = length(robot.body);
body_range = [1, nb];

n_tests = 5;
test_number = 1;
while test_number < n_tests
  base = randi(body_range);
  end_effector = randi(body_range);

  if base ~= end_effector
    q = getRandomConfiguration(robot);
    nPoints = randi([1, 10]);
    points = randn(3, nPoints);
    
    option.grad_method = 'taylorvar';
    kinsol = robot.doKinematics(q, true, false);
    [~, J, dJ] = robot.forwardKinV(kinsol, end_effector, points, rotation_type, base);
    [~, J_geval, dJ_geval] = geval(1, @(q) robot.forwardKinV(robot.doKinematics(q, false, false), end_effector, points, rotation_type, base), q, option);
    valuecheck(J_geval, J, 1e-10);
    valuecheck(dJ_geval, dJ, 1e-10);
    test_number = test_number + 1;
  end
end
end
