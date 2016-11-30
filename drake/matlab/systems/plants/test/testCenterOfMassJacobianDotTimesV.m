function testCenterOfMassJacobianDotTimesV()
floatingBaseParameterizations = {'rpy', 'quat'};

for parameterization = floatingBaseParameterizations
  robot = createAtlas(parameterization{:});
  testAgainstJdotFromGradient(robot);
  testGradients(robot);
  checkMex(robot);
end

end

function testAgainstJdotFromGradient(robot)
nq = robot.getNumPositions();
nv = robot.getNumVelocities();

nTests = 5;
robotnum = 1;
for i = 1 : nTests
  q = getRandomConfiguration(robot);
  v = randn(nv, 1);
  
  options.use_mex = false;
  kinsol = robot.doKinematics(q, v, options);
  Jdot_times_v = robot.centerOfMassJacobianDotTimesV(kinsol, robotnum);
  
  options.use_mex = false;
  options.compute_gradients = true;
  kinsol = robot.doKinematics(q, v, options);
  in_terms_of_qdot = false;

  [~, J, dJ] = robot.getCOM(kinsol, robotnum, in_terms_of_qdot);
  Jdot = reshape(reshape(dJ, [], nq) * kinsol.qd, size(J));
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
  
  options.use_mex = false;
  options.compute_gradients = true;
  kinsol = robot.doKinematics(q, v, options);
  [~, dJdot_times_v] = robot.centerOfMassJacobianDotTimesV(kinsol);
  
  option.grad_method = 'taylorvar';
  options.compute_gradients = false;
  [~, dJdot_times_v_geval] = geval(1, @(q) robot.centerOfMassJacobianDotTimesV(robot.doKinematics(q, v, options)), q, option);
  valuecheck(dJdot_times_v_geval, dJdot_times_v, 1e-10);
end
end

function checkMex(robot)
q = getRandomConfiguration(robot);
v = randn(robot.getNumVelocities(), 1);

options.compute_gradients = true;
options.use_mex = false;
kinsol = robot.doKinematics(q, v, options);
[Jdot_times_v, dJdot_times_v] = robot.centerOfMassJacobianDotTimesV(kinsol);
options.use_mex = true;
kinsol = robot.doKinematics(q, v, options);
[Jdot_times_v_mex, dJdot_times_v_mex] = robot.centerOfMassJacobianDotTimesV(kinsol);
valuecheck(Jdot_times_v, Jdot_times_v_mex);
valuecheck(dJdot_times_v, dJdot_times_v_mex);

end
