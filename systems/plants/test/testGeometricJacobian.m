function testGeometricJacobian

testFallingBrick('quat');
testFallingBrick('quat');
testAtlas('rpy');
testAtlas('quat');

end

function testFallingBrick(floatingType)
options.floating = floatingType;
robot = RigidBodyManipulator('FallingBrick.urdf',options);
checkTransformDerivatives(robot);
checkJacobianGradients(robot);
end

function testAtlas(floatingJointType)

robot = createAtlas(floatingJointType);

checkTransformDerivatives(robot);
checkJacobianGradients(robot);
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
  HBaseDot = kinsol.Tdot{base};
  HBodyDot = kinsol.Tdot{endEffector};
  
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
  q = getRandomConfiguration(robot);
  
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