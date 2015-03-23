function testForwardJacDotTimesV
options.use_new_kinsol = true;
robot = createAtlas('rpy', options);

for rotation_type = 0 : 2
  compareToJacobianGradientMethod(robot, rotation_type);
  testJdotTimesVGradient(robot, rotation_type);
  checkMex(robot, rotation_type);
end

robot = createAtlas('quat', options);
for rotation_type = 0 : 2
  % comparison to Jacobian gradient method is only meaningful for rpy
  % parameterization (derivative of vToqdot needs to be zero)
  testJdotTimesVGradient(robot, rotation_type);
  checkMex(robot, rotation_type);
end
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
    
    valuecheck(Jdot * kinsol.qdot, Jvdot_times_v, computeTolerance(Jvdot_times_v, 1e-3, 1e-6));
    
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
    valuecheck(dJdot_times_v_geval, dJdot_times_v, computeTolerance(dJdot_times_v, 1e-8, 1e-8));
    test_number = test_number + 1;
  end
end

end

function checkMex(robot, rotation_type)
nv = robot.getNumVelocities();
nb = length(robot.body);
body_range = [1, nb];

kinematics_options.compute_gradients = true;

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
    
    kinematics_options.use_mex = false;
    kinsol = robot.doKinematics(q, v, kinematics_options);
    [Jdot_times_v, dJdot_times_v] = robot.forwardJacDotTimesV(kinsol, end_effector, points, rotation_type, base);
    
    kinematics_options.use_mex = true;
    kinsol = robot.doKinematics(q, v, kinematics_options);
    [Jdot_times_v_mex, dJdot_times_v_mex] = robot.forwardJacDotTimesV(kinsol, end_effector, points, rotation_type, base);
    valuecheck(Jdot_times_v_mex, Jdot_times_v, computeTolerance(Jdot_times_v, 1e-8, 1e-8));
    valuecheck(dJdot_times_v_mex, dJdot_times_v, computeTolerance(dJdot_times_v, 1e-8, 1e-8));
    test_number = test_number + 1;
  end
end
end

function ret = computeTolerance(desired, relative_tolerance, min_absolute_tolerance)
if nargin < 3
  min_absolute_tolerance = 0;
end
ret = max(abs(desired(:))) * relative_tolerance;
if ret < min_absolute_tolerance
  ret = min_absolute_tolerance;
end
end

