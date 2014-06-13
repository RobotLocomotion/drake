function testForwardKin

testFallingBrick('rpy');
testFallingBrick('quat');

testAtlas('rpy');
testAtlas('quat');

end

function testFallingBrick(floatingJointType)
options.floating = floatingJointType;
robot = RigidBodyManipulator('FallingBrick.urdf',options);

compareToGeval(robot, 0);
compareToGeval(robot, 1);
compareToGeval(robot, 2);

end

function testAtlas(floatingJointType)

robot = createAtlas(floatingJointType);

compareToGeval(robot, 0);
compareToGeval(robot, 1);
compareToGeval(robot, 2);

end

function compareToGeval(robot, rotation_type)

nb = length(robot.body);
body_range = [2, nb];

computation_time = 0;

n_tests = 10;
epsilon = 1e-10;
for test_number = 1 : n_tests
  q = randn(robot.getNumPositions(), 1); % getRandomConfiguration(robot);
  v = randn(robot.getNumVelocities(), 1);
  kinsol = robot.doKinematics(q, true, false, v, true);
  
  end_effector = randi(body_range);
  nPoints = randi([1, 5]);
  points = randn(3, nPoints);

  tic
  [~, J, dJ] = robot.forwardKin(kinsol, end_effector, points, rotation_type);
  computation_time = computation_time + toc * 1e3;
  
  option.grad_method = 'taylorvar';
  
  [~, ~, J_geval, dJ_geval] = geval(2, @(q) gevalFunction(robot, q, end_effector, points, rotation_type), q, option);
  valuecheck(J_geval, J, epsilon);
  valuecheck(dJ_geval, dJ, epsilon);
end

displayComputationTime = false;
if displayComputationTime
  fprintf('computation time per call: %0.3f ms\n', computation_time / n_tests);
  fprintf('\n');
end
end

function [x, J] = gevalFunction(robot, q, end_effector, points, rotation_type)
kinsol = robot.doKinematics(q,false,false);
[x, J] = robot.forwardKin(kinsol, end_effector, points, rotation_type);
end

% function ret = constraintOrthogonalSubspaceBasis(constraint, q_symbolic, q_numerical)
% J = simplify(jacobian(constraint, q));
% JPerp = null(J);
% [q1, q2, q3, q4] = deal(q(1), q(2), q(3), q(4));
% ret = reshape([-q2./q1,1.0,0.0,0.0,-q3./q1,0.0,1.0,0.0,-q4./q1,0.0,0.0,1.0],[4,3]);
% end