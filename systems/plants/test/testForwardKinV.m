function testForwardKinV

testAtlasGradients();
testAtlasMex('rpy');
% testAtlasMex('quat');

end

function testAtlasGradients()

% test only works for rpy parameterized robots, since in this case qdot = v
robot = createAtlas('rpy');

if robot.use_new_kinsol
  testGradients(robot, 0);
  testGradients(robot, 1);
  testGradients(robot, 2);
end

end

function testAtlasMex(floating_joint_type)
robot = createAtlas(floating_joint_type);
if robot.use_new_kinsol
  checkMex(robot, 0);
  checkMex(robot, 1);
  checkMex(robot, 2);
end
end

function testGradients(robot, rotation_type)
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
    [~, J, dJ] = robot.forwardKinV(kinsol, end_effector, points, rotation_type, base);
    [~, J_geval, dJ_geval] = geval(1, @(q) robot.forwardKinV(robot.doKinematics(q, [], [], [], kinsol_options_geval), end_effector, points, rotation_type, base), q, option);
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
  base = 1; randi(body_range);
  end_effector = randi(body_range);
  
  q = getRandomConfiguration(robot);
  nPoints = randi([1, 10]);
  points = randn(3, nPoints);
  
  kinsol_options.use_mex = false;
  kinsol_options.compute_gradients = true;
  kinsol = robot.doKinematics(q, [], [], [], kinsol_options);
  [x, J, dJ] = robot.forwardKinV(kinsol, end_effector, points, rotation_type, base);
  
  kinsol_options.use_mex = true;
  kinsol_mex = robot.doKinematics(q, [], [], [], kinsol_options);
%   [x_mex, J_mex, dJ_mex] = robot.forwardKinV(kinsol_mex, end_effector, points, rotation_type, base);
  [x_mex, J_mex] = robot.forwardKinV(kinsol_mex, end_effector, points, rotation_type, base);
  valuecheck(x_mex, x);
  valuecheck(J_mex, J);
%   valuecheck(dJ_mex, dJ);
end
end
