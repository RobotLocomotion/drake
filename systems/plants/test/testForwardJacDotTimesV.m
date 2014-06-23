function testForwardJacDotTimesV

testAtlas();

end

function testAtlas()

% test only works for rpy parameterized robots, since in this case qdot = v
robot = createAtlas('rpy');

compareToJacobianGradientMethod(robot, 0);
compareToJacobianGradientMethod(robot, 1);
compareToJacobianGradientMethod(robot, 2);

testJdotTimesVGradient(robot, 0);
testJdotTimesVGradient(robot, 1);
testJdotTimesVGradient(robot, 2);

end

function compareToJacobianGradientMethod(robot, rotation_type)

nv = robot.getNumVelocities();
nb = length(robot.body);
body_range = [1, nb];


n_tests = 10;
test_number = 1;
epsilon = 1e-3;
while test_number < n_tests
  q = getRandomConfiguration(robot);
  v = randn(nv, 1);
  kinsol = robot.doKinematics(q, true, false, v, true);
  
  base = randi(body_range);
  end_effector = randi(body_range);
  if base ~= end_effector
    nPoints = randi([1, 10]);
    points = randn(3, nPoints);
    
    [~, J, dJ] = robot.forwardKinV(kinsol, end_effector, points, rotation_type, base);
    qdot = kinsol.vToqdot * v;
    Jdot = reshape(reshape(dJ, numel(J), []) * qdot, size(J));
    Jvdot_times_v = robot.forwardJacDotTimesV(kinsol, end_effector, points, rotation_type, base);
    
    valuecheck(Jdot * v, Jvdot_times_v, epsilon);
    
    test_number = test_number + 1;
  end
end

end

function testJdotTimesVGradient(robot, rotation_type)
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
    kinsol = robot.doKinematics(q, true, false, v, true);
    [~, dJdot_times_v] = robot.forwardJacDotTimesV(kinsol, end_effector, points, rotation_type, base);
    [~, dJdot_times_v_geval] = geval(1, @(q) robot.forwardJacDotTimesV(robot.doKinematics(q, false, false, v), end_effector, points, rotation_type, base), q, option);
    valuecheck(dJdot_times_v_geval, dJdot_times_v, 1e-10);
    test_number = test_number + 1;
  end
end

end
