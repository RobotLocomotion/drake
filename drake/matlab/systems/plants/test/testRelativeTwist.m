function testRelativeTwist()
testAtlas('rpy');
testAtlas('quat');
end

function testAtlas(floatingJointType)
robot = createAtlas(floatingJointType);
checkAgainstJacobianTimesJointVelocities(robot);
checkGradients(robot);
end

function checkAgainstJacobianTimesJointVelocities(robot)
nBodies = length(robot.body);
bodyRange = [1, nBodies];

nTests = 50;
test_number = 0;
while test_number < nTests
  q = getRandomConfiguration(robot);
  v = randn(robot.getNumVelocities(), 1);
  options.use_mex = false;
  kinsol = robot.doKinematics(q, v, options);
  
  base = randi(bodyRange);
  end_effector = randi(bodyRange);
  expressed_in = randi(bodyRange);
  
  if base ~= end_effector
    twist = relativeTwist(kinsol, base, end_effector, expressed_in);
    [jacobian, v_indices] = robot.geometricJacobian(kinsol, base, end_effector, expressed_in);
    twistViaJacobian = jacobian * (v(v_indices));
    valuecheck(twistViaJacobian, twist, 1e-12);
    test_number = test_number + 1;
  end
end
end

function checkGradients(robot)
nBodies = length(robot.body);
bodyRange = [1, nBodies];

nTests = 10;
test_number = 0;
while test_number < nTests
  base = randi(bodyRange);
  end_effector = randi(bodyRange);
  expressed_in = randi(bodyRange);
  
  if base ~= end_effector
    q = getRandomConfiguration(robot);
    v = randn(robot.getNumVelocities());
    options.use_mex = false;
    options.compute_gradients = true;
    kinsol = robot.doKinematics(q, v, options);
    [~, dtwist] = relativeTwist(kinsol, base, end_effector, expressed_in);
    
    option.grad_method = 'taylorvar';
    [~, dtwistCheck] = geval(1, @(q) gevalFunction(robot, q, v, base, end_effector, expressed_in), q, option);
    
    valuecheck(dtwistCheck, dtwist, 1e-12);
    test_number = test_number + 1;
  end
end
end

function ret = gevalFunction(robot, q, v, base, end_effector, expressed_in)
options.use_mex = false;
kinsol = robot.doKinematics(q, v, options);
ret = relativeTwist(kinsol, base, end_effector, expressed_in);
end