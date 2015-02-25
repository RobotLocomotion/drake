function testForwardKin

testTwoFallingBricks('rpy');
testAtlas('rpy');

options.use_new_kinsol = true;
testTwoFallingBricks('rpy',options);
testAtlas('rpy',options);
testTwoFallingBricks('quat',options);
testAtlas('quat',options);

end

function testTwoFallingBricks(floatingJointType,options)
options.floating = floatingJointType;
robot = RigidBodyManipulator('FallingBrick.urdf',options);
robot = robot.addRobotFromURDF('FallingBrick.urdf', [], [], options);
for rotation_type = 0 : 2
  checkGradients(robot, rotation_type);  
  checkMex(robot, rotation_type);
  checkJacobianWithAndWithoutGradients(robot, rotation_type);
end
end

function testAtlas(floatingJointType,options)
if nargin<2, options=struct(); end
robot = createAtlas(floatingJointType,options);

for rotation_type = 0 : 2
  checkGradients(robot, rotation_type);  
  checkMex(robot, rotation_type);
  checkJacobianWithAndWithoutGradients(robot, rotation_type);
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
    
    geval_options.grad_method = 'taylorvar';
    kinsol_options.use_mex = false;
    kinsol_options.compute_gradients = true;
    kinsol_options_geval.use_mex = kinsol_options.use_mex;
    kinsol = robot.doKinematics(q, [], kinsol_options);
    
    forwardKin_options.rotation_type = rotation_type;
    forwardKin_options.base_or_frame_id = base;
    
    if robot.use_new_kinsol || rotation_type == 0
      [~, J, dJ] = robot.forwardKin(kinsol, end_effector, points, forwardKin_options);
      [~, J_geval, dJ_geval] = geval(1, @(q) robot.forwardKin(robot.doKinematics(q, [], kinsol_options_geval), end_effector, points, forwardKin_options), q, geval_options);
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
options.rotation_type = rotation_type;

n_tests = 5;
for i = 1 : n_tests
  end_effector = randi(body_range);

  if robot.use_new_kinsol
    base = randi(body_range);
    in_terms_of_qdot_options = [true false];
  else
    base = 1;
    in_terms_of_qdot_options = true;
  end
  
  q = getRandomConfiguration(robot);
  nPoints = randi([1, 10]);
  points = randn(3, nPoints);
  
  kinsol_options.use_mex = false;
  kinsol_options.compute_gradients = true;
  kinsol = robot.doKinematics(q, [], kinsol_options);
  kinsol_options.use_mex = true;
  kinsol_mex = robot.doKinematics(q, [], kinsol_options);
  
  for in_terms_of_qdot = in_terms_of_qdot_options
    options.in_terms_of_qdot = in_terms_of_qdot;
    options.base_or_frame_id = base;
    if robot.use_new_kinsol || rotation_type == 0
      [x, J, dJ] = robot.forwardKin(kinsol, end_effector, points, options);
      [x_mex, J_mex, dJ_mex] = robot.forwardKin(kinsol_mex, end_effector, points, options);
    else
      [x, J] = robot.forwardKin(kinsol, end_effector, points, options);
      [x_mex, J_mex] = robot.forwardKin(kinsol_mex, end_effector, points, options);
    end
    
    valuecheck(x_mex, x);
    valuecheck(J_mex, J);
    if robot.use_new_kinsol || rotation_type == 0
      valuecheck(dJ_mex, dJ);
    end
  end
end
end

function checkJacobianWithAndWithoutGradients(robot, rotation_type)
nb = length(robot.body);
body_range = [1, nb];
options.rotation_type = rotation_type;

n_tests = 5;
for i = 1 : n_tests
  end_effector = randi(body_range);

  if robot.use_new_kinsol
    base = randi(body_range);
    in_terms_of_qdot_options = [true false];
  else
    base = 1;
    in_terms_of_qdot_options = true;
  end
  
  q = getRandomConfiguration(robot);
  nPoints = randi([1, 10]);
  points = randn(3, nPoints);
  kinsol_options.use_mex = true;

  for in_terms_of_qdot = in_terms_of_qdot_options
    options.in_terms_of_qdot = in_terms_of_qdot;
    options.base_or_frame_id = base;
    
    kinsol_options.compute_gradients = false;
    kinsol = robot.doKinematics(q, [], kinsol_options);
    [x, J] = robot.forwardKin(kinsol, end_effector, points, options);
    
    kinsol_options.compute_gradients = true;
    kinsol_with_gradients = robot.doKinematics(q, [], kinsol_options);
    [x_with_gradients, J_with_gradients] = robot.forwardKin(kinsol_with_gradients, end_effector, points, options);
    
    valuecheck(x_with_gradients, x);
    valuecheck(J_with_gradients, J);
  end
end

end
