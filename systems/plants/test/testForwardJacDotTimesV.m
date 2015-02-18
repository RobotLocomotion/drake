function testForwardJacDotTimesV
% tests only meaningful for rpy parameterization (derivative of
% kinsol.vToqdot needs to be zero)
options.use_new_kinsol = true;
robot = createAtlas('rpy', options);

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

kinematics_options.use_mex = false;
kinematics_options.compute_gradients = true;
forwardkin_options.rotation_type = rotation_type;

n_tests = 10;
test_number = 1;
epsilon = 1e-3;

while test_number < n_tests
  q = getRandomConfiguration(robot);
  v = randn(nv, 1);
  
  kinsol = robot.doKinematics(q, v, kinematics_options);
  
  base = randi(body_range);
  end_effector = randi(body_range);
  forwardkin_options.base_or_frame_id = base;
  if base ~= end_effector
    nPoints = randi([1, 10]);
    points = randn(3, nPoints);
    
    [~, J, dJ] = robot.forwardKin(kinsol, end_effector, points, forwardkin_options);
    Jdot = reshape(reshape(dJ, numel(J), []) * kinsol.qdot, size(J));
    Jvdot_times_v = robot.forwardJacDotTimesV(kinsol, end_effector, points, rotation_type, base);
    
%     valuecheck(Jdot * qdot, Jvdot_times_v, epsilon);
    valuecheck(Jdot * kinsol.qdot - Jvdot_times_v, zeros(size(Jdot, 1), 1), epsilon);
    
    test_number = test_number + 1;
  end
end

end

function testJdotTimesVGradient(robot, rotation_type)
nv = robot.getNumVelocities();
nb = length(robot.body);
body_range = [1, nb];

geval_options.grad_method = 'taylorvar';
kinematics_options.use_mex = false;

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
    
    kinematics_options.compute_gradients = true;
    kinsol = robot.doKinematics(q, v, kinematics_options);
    [~, dJdot_times_v] = robot.forwardJacDotTimesV(kinsol, end_effector, points, rotation_type, base);
    kinematics_options.compute_gradients = false;
    [~, dJdot_times_v_geval] = geval(1, @(q) robot.forwardJacDotTimesV(robot.doKinematics(q, v, kinematics_options), end_effector, points, rotation_type, base), q, geval_options);
    valuecheck(dJdot_times_v_geval, dJdot_times_v, 1e-10);
    test_number = test_number + 1;
  end
end

end
