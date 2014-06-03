function testGeometricJacobianDotV()
testAtlas('rpy');
testAtlas('quat');

end

function testAtlas(floatingJointType)
robot = createAtlas(floatingJointType);
testVersusNumericalDifferentiation(robot);
testGradient(robot);
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
  if base ~= end_effector
    % random state
    q = getRandomConfiguration(robot);
    v = randn(robot.getNumVelocities(), 1);
    
    % compute Jacobian, JDotV
    kinsol = robot.doKinematics(q, false, false, v, true);
    [J0, vIndices] = robot.geometricJacobian(kinsol, base, end_effector, expressed_in);
    Jdot_times_v = robot.geometricJacobianDotV(kinsol, base, end_effector, expressed_in);
    
    % integrate
    q = q + kinsol.vToqdot * v * dt;
    
    % update kinematics, Jacobian
    kinsolNew = robot.doKinematics(q, false, false, v);
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
  
  if base ~= end_effector
    q = getRandomConfiguration(robot);
    v = randn(robot.getNumVelocities(), 1);
    
    kinsol = robot.doKinematics(q, true, false, v, true);
    [~, dJdot_times_v] = robot.geometricJacobianDotV(kinsol, base, end_effector, expressed_in);
    
    option.grad_method = 'taylorvar';
    [~, dJdot_times_v_geval] = geval(1, @(q) gevalFunction(robot, q, v, base, end_effector, expressed_in), q, option);
    
    valuecheck(dJdot_times_v_geval, dJdot_times_v, 1e-10);
    
    test_number = test_number + 1;
  end
end
end

function Jdot_times_v = gevalFunction(robot, q, v, base, end_effector, expressed_in)
kinsol = robot.doKinematics(q, false, false, v, true);
Jdot_times_v = robot.geometricJacobianDotV(kinsol, base, end_effector, expressed_in);
end