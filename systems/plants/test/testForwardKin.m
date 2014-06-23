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

n_tests = 2;
epsilon = 1e-10;
for test_number = 1 : n_tests
  q = randn(robot.getNumPositions(), 1); % getRandomConfiguration(robot);
  kinsol = robot.doKinematics(q, true, false);
  
  end_effector = randi(body_range);
  nPoints = randi([1, 5]);
  points = randn(3, nPoints);

  [~, J, dJ] = robot.forwardKin(kinsol, end_effector, points, rotation_type);
  
  option.grad_method = 'taylorvar';
  [~, J_geval, dJ_geval] = geval(1, @(q) robot.forwardKin(robot.doKinematics(q,false,false), end_effector, points, rotation_type), q, option);
  valuecheck(J_geval, J, epsilon);
  valuecheck(dJ_geval, dJ, epsilon);
end

end

% function ret = constraintOrthogonalSubspaceBasis(constraint, q_symbolic, q_numerical)
% J = simplify(jacobian(constraint, q));
% JPerp = null(J);
% [q1, q2, q3, q4] = deal(q(1), q(2), q(3), q(4));
% ret = reshape([-q2./q1,1.0,0.0,0.0,-q3./q1,0.0,1.0,0.0,-q4./q1,0.0,0.0,1.0],[4,3]);
% end