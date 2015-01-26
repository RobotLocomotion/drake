function testForwardKin

testFallingBrick('rpy');
% testFallingBrick('quat');

testAtlas('rpy');
% testAtlas('quat');

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
  base = randi(body_range);
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
    [~, J, dJ] = robot.forwardKin(kinsol, end_effector, points, rotation_type);
    [~, J_geval, dJ_geval] = geval(1, @(q) robot.forwardKin(robot.doKinematics(q, [], [], [], kinsol_options_geval), end_effector, points, rotation_type), q, option);
    valuecheck(J_geval, J, 1e-10);
    valuecheck(dJ_geval, dJ, 1e-10);
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
  
  q = getRandomConfiguration(robot);
  nPoints = randi([1, 10]);
  points = randn(3, nPoints);
  
  kinsol_options.use_mex = false;
  kinsol_options.compute_gradients = true;
  kinsol = robot.doKinematics(q, [], [], [], kinsol_options);
  [x, J, dJ] = robot.forwardKin(kinsol, end_effector, points, rotation_type);
  
  kinsol_options.use_mex = true;
  kinsol_mex = robot.doKinematics(q, [], [], [], kinsol_options);
  [x_mex, J_mex, dJ_mex] = robot.forwardKin(kinsol_mex, end_effector, points, rotation_type);
  valuecheck(x_mex, x);
  valuecheck(J_mex, J);
  valuecheck(dJ_mex, dJ);
end
end