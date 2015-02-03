function testForwardKin
testFallingBrick('rpy');
testAtlas('rpy');

if RigidBodyManipulator.use_new_kinsol
  testFallingBrick('quat');
  testAtlas('quat');
end

end

function testFallingBrick(floatingJointType)
options.floating = floatingJointType;
robot = RigidBodyManipulator('FallingBrick.urdf',options);
for rotation_type = 0 : 2
  checkGradients(robot, rotation_type);  
  checkMex(robot, rotation_type);
end
end

function testAtlas(floatingJointType)
robot = createAtlas(floatingJointType);
for rotation_type = 0 : 2
  checkGradients(robot, rotation_type);  
  checkMex(robot, rotation_type);
end
end

function checkGradients(robot, rotation_type)
nb = length(robot.body);
body_range = [1, nb];

n_tests = 5;
test_number = 1;
while test_number < n_tests
  if robot.use_new_kinsol
    base = randi(body_range);
  else
    base = 1;
  end
  end_effector = randi(body_range);
  
  if base ~= end_effector
    q = getRandomConfiguration(robot);
    nPoints = randi([1, 10]);
    points = randn(3, nPoints);
    
    option.grad_method = 'taylorvar';
    kinsol_options.use_mex = false;
    kinsol_options.compute_gradients = true;
    kinsol_options_geval.use_mex = kinsol_options.use_mex;
    kinsol = robot.doKinematics(q, [], [], [], kinsol_options);
    if robot.use_new_kinsol || rotation_type == 0
      [~, J, dJ] = robot.forwardKin(kinsol, end_effector, points, rotation_type, base);
      [~, J_geval, dJ_geval] = geval(1, @(q) robot.forwardKin(robot.doKinematics(q, [], [], [], kinsol_options_geval), end_effector, points, rotation_type, base), q, option);
      valuecheck(J_geval, J, 1e-10);
      valuecheck(dJ_geval, dJ, 1e-10);
    end

    test_number = test_number + 1;
  end
end
end

function checkMex(robot, rotation_type)
nb = length(robot.body);
body_range = [1, nb];

n_tests = 5;
for i = 1 : n_tests
  end_effector = randi(body_range);

  if robot.use_new_kinsol
    base = randi(body_range);
  else
    base = 1;
  end
  
  q = getRandomConfiguration(robot);
  nPoints = randi([1, 10]);
  points = randn(3, nPoints);
  
  kinsol_options.use_mex = false;
  kinsol_options.compute_gradients = true;
  kinsol = robot.doKinematics(q, [], [], [], kinsol_options); 
  kinsol_options.use_mex = true;
  kinsol_mex = robot.doKinematics(q, [], [], [], kinsol_options);
  
  if robot.use_new_kinsol || rotation_type == 0
    [x, J, dJ] = robot.forwardKin(kinsol, end_effector, points, rotation_type, base);
    [x_mex, J_mex, dJ_mex] = robot.forwardKin(kinsol_mex, end_effector, points, rotation_type, base);
  else
    [x, J] = robot.forwardKin(kinsol, end_effector, points, rotation_type);
    [x_mex, J_mex] = robot.forwardKin(kinsol_mex, end_effector, points, rotation_type, base);
  end
  
  valuecheck(x_mex, x);
  valuecheck(J_mex, J);
  if robot.use_new_kinsol || rotation_type == 0
    valuecheck(dJ_mex, dJ);
  end
end
end