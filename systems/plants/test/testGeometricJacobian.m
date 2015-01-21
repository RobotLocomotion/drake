function testGeometricJacobian
testFallingBrick('rpy');
% testFallingBrick('quat'); % FLOATINGBASE TODO: reenable
testAtlas('rpy');
% testAtlas('quat'); % FLOATINGBASE TODO: reenable
end

function testFallingBrick(floatingType)
options.floating = floatingType;
robot = RigidBodyManipulator('FallingBrick.urdf',options);
checkTransformDerivatives(robot);
if robot.use_new_kinsol
  checkJacobianGradients(robot);
  checkMex(robot);
end
end

function testAtlas(floatingJointType)
robot = createAtlas(floatingJointType);
checkTransformDerivatives(robot);
if robot.use_new_kinsol
  checkJacobianGradients(robot);
  checkMex(robot);
end
end

function checkTransformDerivatives(robot)
nv = robot.getNumVelocities();
nBodies = length(robot.body);

bodyRange = [1, nBodies];
nTests = 50;
for i = 1 : nTests
  q = getRandomConfiguration(robot);
  v = randn(nv, 1);
  kinsol = robot.doKinematics(q,false,false, v);
  
  base = randi(bodyRange);
  endEffector = randi(bodyRange);
  expressedIn = base;
  
  [J, vIndices] = robot.geometricJacobian(kinsol, base, endEffector, expressedIn);
  if base == endEffector
    twist = J * zeros(0, 1);
  else
    twist = J * (v(vIndices));
  end
  
  HBase = kinsol.T{base};
  HBody = kinsol.T{endEffector};
  if robot.use_new_kinsol
    Tdot = computeTdots(kinsol.T, kinsol.twists);
  else
    Tdot = kinsol.Tdot;
  end
  HBaseDot = Tdot{base};
  HBodyDot = Tdot{endEffector};
  
  HBodyToBase = HBase \ HBody;
  transformDot = twistToTildeForm(twist) * HBodyToBase;
  transformDotForwardKin = relativeTdot(HBase, HBody, HBaseDot, HBodyDot);
  valuecheck(transformDot, transformDotForwardKin, 1e-12);
end
end

function checkJacobianGradients(robot)
nb = length(robot.body);

bodyRange = [1, nb];
nTests = 10;

for test = 1 : nTests
  %   q = getRandomConfiguration(robot);
  q = randn(robot.getNumPositions(), 1);
  
  base = randi(bodyRange);
  end_effector = randi(bodyRange);
  expressed_in = randi(bodyRange);
  
  kinsol = robot.doKinematics(q,true,false);
  [~, ~, dJdq] = robot.geometricJacobian(kinsol, base, end_effector, expressed_in);
  option.grad_method = 'numerical';
  %   option.grad_method = 'taylorvar';
  [~, dJdq_geval] = geval(1, @(q) gevalFunction(robot, q, base, end_effector, expressed_in), q, option);
  valuecheck(dJdq_geval, dJdq, 1e-5);
  
end
end

function checkMex(robot)
nb = length(robot.body);

bodyRange = [1, nb];
nTests = 10;

for test = 1 : nTests
  %   q = getRandomConfiguration(robot);
  q = randn(robot.getNumPositions(), 1);
  
  base = randi(bodyRange);
  end_effector = randi(bodyRange);
  expressed_in = randi(bodyRange);
  
  kinsol = robot.doKinematics(q,false,false);
  [J, v_indices, dJdq] = robot.geometricJacobian(kinsol, base, end_effector, expressed_in);
  
  kinsol_mex = robot.doKinematics(q, false, true);
  [J_mex, v_indices_mex, dJdq_mex] = robot.geometricJacobian(kinsol_mex, base, end_effector, expressed_in);
  
  valuecheck(J, J_mex);
  valuecheck(v_indices, v_indices_mex);
  valuecheck(dJdq, dJdq_mex);
end
end

function ret = relativeTdot(TBase, TBody, TBaseDot, TBodyDot)
% TRelative = inv(TBase) * TBody
% d/dt(TRelative) = d/dt(inv(TBase) * TBody) + inv(TBase) * TBodyDot
% d/dt(inv(Tbase) = -inv(TBase) * TBaseDot * inv(Tbase)

invTBasedot = -TBase \ TBaseDot / TBase;
ret = invTBasedot * TBody + TBase \ TBodyDot;

end

function J = gevalFunction(robot, q, base, end_effector, expressed_in)
kinsol = robot.doKinematics(q,false,false);
J = robot.geometricJacobian(kinsol, base, end_effector, expressed_in);
J = J(:);
end

function ret = twistToTildeForm(twist)
omega = twist(1 : 3);
v = twist(4 : 6);
ret = [vectorToSkewSymmetric(omega), v;
  zeros(1, 4)];
end