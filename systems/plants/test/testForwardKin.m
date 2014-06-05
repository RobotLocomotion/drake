function testForwardKin

testFallingBrick('quat');

% testAtlas('rpy');
testAtlas('quat');

end

function testFallingBrick(floatingJointType)
options.floating = floatingJointType;
robot = RigidBodyManipulator('FallingBrick.urdf',options);

compareToNumerical(robot, 0);
compareToNumerical(robot, 1);
compareToNumerical(robot, 2);

end

function testAtlas(floatingJointType)

robot = createAtlas(floatingJointType);

compareToNumerical(robot, 0);
compareToNumerical(robot, 1);
compareToNumerical(robot, 2);

end

function compareToNumerical(robot, rotation_type)

nb = length(robot.body);
body_range = [2, nb];

computation_time = 0;

n_tests = 10;
epsilon = 1e-3;
for test_number = 1 : n_tests
%   q = getRandomConfiguration(robot);
  q = zeros(robot.getNumPositions(), 1);
  q(4) = 1;
  kinsol = robot.doKinematics(q, false, false);
  
  end_effector = randi(body_range);
  nPoints = 1; %randi([1, 10]);
  points = randn(3, nPoints);
  
  tic
  [~, J] = robot.forwardKin(kinsol, end_effector, points, rotation_type);
  computation_time = computation_time + toc * 1e3;
  
  option.grad_method = 'taylorvar';
  
  [~, J_numerical] = geval(1, @(q) gevalFunction(robot, q, end_effector, points, rotation_type), q, option);
  valuecheck(J_numerical, J, epsilon);
  
end

displayComputationTime = false;
if displayComputationTime
  fprintf('computation time per call: %0.3f ms\n', computation_time / n_tests);
  fprintf('\n');
end
end

function x = gevalFunction(robot, q, end_effector, points, rotation_type)
kinsol = robot.doKinematics(q,false,false);
x = robot.forwardKin(kinsol, end_effector, points, rotation_type);
x = x(:);
end
