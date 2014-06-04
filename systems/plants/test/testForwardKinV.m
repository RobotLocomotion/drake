function testForwardKinV

testAtlas('rpy');
% testAtlas('quat'); % TODO: generating robot currently results in an error.

end

function testAtlas(floatingJointType)

robot = createAtlas(floatingJointType);

compareToNumerical(robot, 0);
compareToNumerical(robot, 1);
compareToNumerical(robot, 2);

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
      valuecheck(J(:, col), dxdq_numerical(:), epsilon);
    end
    
    q_delta = q + delta * v;
    kinsol = robot.doKinematics(q_delta, false, false);
    [~, J_delta] = robot.forwardKinV(kinsol, end_effector, points, rotation_type, base);
    Jdot_numerical = (J_delta - J) / delta;
    valuecheck(Jvdot_times_v, Jdot_numerical * v, epsilon);
    
    test_number = test_number + 1;
  end
end

displayComputationTime = false;
if displayComputationTime
  fprintf('computation time per call: %0.3f ms\n', computation_time / n_tests);
  fprintf('\n');
end
end