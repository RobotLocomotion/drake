function testGeometricJacobianDotTimesV()
testAtlas('rpy');
testAtlas('quat');

end

function testAtlas(floatingJointType)
robot = createAtlas(floatingJointType, options);
testVersusNumericalDifferentiation(robot);
testGradient(robot);
checkMex(robot);
end

function testVersusNumericalDifferentiation(robot)
dt = 1e-8;

ntests = 50;
nb = length(robot.body);
body_range = [1, nb];
test_number = 1;
while test_number <= ntests
  end_effector = randi(body_range);
  base = randi(body_range);
  expressed_in = randi(body_range);
  % random state
  q = getRandomConfiguration(robot);
  v = randn(robot.getNumVelocities(), 1);
  
  % compute Jacobian, JDotV
  options.use_mex = false;
  kinsol = robot.doKinematics(q, v, options);
  [J0, vIndices] = robot.geometricJacobian(kinsol, base, end_effector, expressed_in);
  Jdot_times_v = robot.geometricJacobianDotTimesV(kinsol, base, end_effector, expressed_in);
  
  % integrate
  q = q + kinsol.qd * dt;
  
  % update kinematics, Jacobian
  options.use_mex = true;
  kinsolNew = robot.doKinematics(q, [], options);
  J1 = robot.geometricJacobian(kinsolNew, base, end_effector, expressed_in);
  
  % compute JDotV through numerical differentiation
  Jdot_times_v_numerical = (J1 - J0) / dt * v(vIndices);
  
  %     disp(testNumber);
  % check
  [same, errstr] = valuecheck(Jdot_times_v, Jdot_times_v_numerical, 1e-5);
  
  if ~same
    fprintf('base: %d\n', base);
    fprintf('endEffector: %d\n', end_effector);
    fprintf('expressedIn: %d\n', expressed_in);
    error(errstr);
  end
  
  test_number = test_number + 1;
end
end

function testGradient(robot)
nb = length(robot.body);
body_range = [1, nb];

test_number = 1;
ntests = 5;
while test_number <= ntests
  end_effector = randi(body_range);
  base = randi(body_range);
  expressed_in = randi(body_range);
  
  q = getRandomConfiguration(robot);
  v = randn(robot.getNumVelocities(), 1);
  
  options.use_mex = false;
  options.compute_gradients = true;
  kinsol = robot.doKinematics(q, v, options);
  [~, dJdot_times_v] = robot.geometricJacobianDotTimesV(kinsol, base, end_effector, expressed_in);
  
  options.compute_gradients = false;
  option.grad_method = 'taylorvar';
  [~, dJdot_times_v_geval] = geval(1, @(q) gevalFunction(robot, q, v, base, end_effector, expressed_in), q, option);
  
  valuecheck(dJdot_times_v_geval, dJdot_times_v, 1e-10);
  
  test_number = test_number + 1;
end
end

function Jdot_times_v = gevalFunction(robot, q, v, base, end_effector, expressed_in)
options.use_mex = false;
kinsol = robot.doKinematics(q, v, options);
Jdot_times_v = robot.geometricJacobianDotTimesV(kinsol, base, end_effector, expressed_in);
end

function checkMex(robot)
nb = length(robot.body);
body_range = [1, nb];

ntests = 5;
options.compute_gradients = true;
for i = 1 : ntests
  end_effector = randi(body_range);
  base = randi(body_range);
  expressed_in = randi(body_range);
  
  q = getRandomConfiguration(robot);
  v = randn(robot.getNumVelocities(), 1);
  
  options.use_mex = false;
  kinsol = robot.doKinematics(q, v, options);
  [Jdot_times_v, dJdot_times_v] = robot.geometricJacobianDotTimesV(kinsol, base, end_effector, expressed_in);
  
  options.use_mex = true;
  kinsol = robot.doKinematics(q, v, options);
  [Jdot_times_v_mex, dJdot_times_v_mex] = robot.geometricJacobianDotTimesV(kinsol, base, end_effector, expressed_in);
  
  valuecheck(Jdot_times_v, Jdot_times_v_mex);
  valuecheck(dJdot_times_v, dJdot_times_v_mex);
end
end