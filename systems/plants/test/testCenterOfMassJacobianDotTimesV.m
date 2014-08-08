function testCenterOfMassJacobianDotTimesV()
floatingBaseParameterizations = {'rpy', 'quat'};

for parameterization = floatingBaseParameterizations
  robot = createAtlas(parameterization{:});
  testAgainstJdotFromGradient(robot);
  testGradients(robot);
end
end

function testAgainstJdotFromGradient(robot)
nq = robot.getNumPositions();
nv = robot.getNumVelocities();

nTests = 5;
for i = 1 : nTests
  q = getRandomConfiguration(robot);
  v = randn(nv, 1);
  
  kinsol = robot.doKinematics(q,false,false, v);
  Jdot_times_v = robot.centerOfMassJacobianDotTimesV(kinsol);
  
  kinsol = robot.doKinematics(q,true,false, v);
  [~, J, dJ] = robot.centerOfMassV(kinsol);
  qdot = kinsol.vToqdot * kinsol.v;
  Jdot = reshape(reshape(dJ, [], nq) * qdot, size(J));
  Jdot_times_v_check = Jdot * v;

  valuecheck(Jdot_times_v_check, Jdot_times_v, 1e-10);
end
end

function testGradients(robot)
nv = robot.getNumVelocities();

nTests = 5;
for i = 1 : nTests
  q = getRandomConfiguration(robot);
  v = randn(nv, 1);
  
  kinsol = robot.doKinematics(q,true,false, v);
  [~, dJdot_times_v] = robot.centerOfMassJacobianDotTimesV(kinsol);
  
  option.grad_method = 'taylorvar';
  [~, dJdot_times_v_geval] = geval(1, @(q) robot.centerOfMassJacobianDotTimesV(robot.doKinematics(q,false,false, v)), q, option);
  valuecheck(dJdot_times_v_geval, dJdot_times_v, 1e-10);
end
end