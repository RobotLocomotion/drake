function testForwardKinV

testAtlas();

end

function testAtlas()

robot = createAtlas('rpy');

compareToNumerical(robot, 0);
compareToNumerical(robot, 1);
compareToNumerical(robot, 2);

testGradient(robot, 0);
testGradient(robot, 1);
testGradient(robot, 2);

end

function compareToNumerical(robot, rotation_type)

nv = robot.getNumVelocities();
nb = length(robot.body);
body_range = [1, nb];

computation_time = 0;

n_tests = 10;
test_number = 1;
delta = 1e-10;
epsilon = 1e-3;
while test_number < n_tests
  q = getRandomConfiguration(robot);
  v = randn(nv, 1);
  kinsol = robot.doKinematics(q, false, false, v, true);
  
  base = randi(body_range);
  end_effector = randi(body_range);
  if base ~= end_effector
    nPoints = randi([1, 10]);
    points = randn(3, nPoints);
    
    tic
    [x, J, Jvdot_times_v] = robot.forwardKinV(kinsol, end_effector, points, rotation_type, base);
    computation_time = computation_time + toc * 1e3;
    
    for col = 1 : length(q)
      q_delta = q;
      q_delta(col) = q_delta(col) + delta;
      kinsol = robot.doKinematics(q_delta, false, false);
      x_delta = robot.forwardKinV(kinsol, end_effector, points, rotation_type, base);
      dxdq_numerical = (x_delta - x) / delta;
      valuecheck(dxdq_numerical(:), J(:, col), epsilon);
    end

    q_delta = q + delta * v;
    kinsol = robot.doKinematics(q_delta, false, false);
    [~, J_delta] = robot.forwardKinV(kinsol, end_effector, points, rotation_type, base);
    Jdot_numerical = (J_delta - J) / delta;
    valuecheck(Jdot_numerical * v, Jvdot_times_v, epsilon);
    
    test_number = test_number + 1;
  end
end

displayComputationTime = false;
if displayComputationTime
  fprintf('computation time per call: %0.3f ms\n', computation_time / n_tests);
  fprintf('\n');
end
end

function testGradient(robot, rotation_type)
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
    [~, ~, ~, dJ, dJdot_times_v] = robot.forwardKinV(kinsol, end_effector, points, rotation_type, base);
    [~, ~, dJ_geval, dJdot_times_v_geval] = geval(2, @(q) gevalFunction(robot, q, v, end_effector, points, rotation_type, base), q, option);
    valuecheck(dJ_geval, dJ, 1e-10);
    valuecheck(dJdot_times_v_geval, dJdot_times_v, 1e-10);
    test_number = test_number + 1;
  end
end

end

function [J, Jdot_times_v] = gevalFunction(robot, q, v, end_effector, points, rotation_type, base)
kinsol = robot.doKinematics(q, false, false, v, true);
[~, J, Jdot_times_v] = robot.forwardKinV(kinsol, end_effector, points, rotation_type, base);
end