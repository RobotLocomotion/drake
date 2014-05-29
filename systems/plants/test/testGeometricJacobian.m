function testGeometricJacobian

testAtlas('rpy');
testAtlas('quat');

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

function checkJacobianGradients(r)
nq = r.getNumPositions();
nv = r.getNumVelocities();
nb = length(r.body);

bodyRange = [1, nb];
nTests = 10;

for test = 1 : nTests
  q = getRandomConfiguration(r);
  v = randn(nv, 1);
  
  base = randi(bodyRange);
  endEffector = randi(bodyRange);
  expressedIn = randi(bodyRange);
  
  kinsol = r.doKinematics(q,true,false, v);
  [J, ~, dJdq] = r.geometricJacobian(kinsol, base, endEffector, expressedIn);

  delta = 1e-7;
  for i = 1 : nq
    dq = zeros(nq, 1);
    dq(i) = delta;
    kinsol_delta = doKinematics(r, q + dq, false, false);
    J_delta = r.geometricJacobian(kinsol_delta, base, endEffector, expressedIn);
    dJdqiNumerical = (J_delta - J) / delta;
    
    for j = 2 : nb
      valuecheck(dJdqiNumerical(:), dJdq(:, i), 1e-5);
    end
  end
end
  

end

function ret = relativeTdot(TBase, TBody, TBaseDot, TBodyDot)
% TRelative = inv(TBase) * TBody
% d/dt(TRelative) = d/dt(inv(TBase) * TBody) + inv(TBase) * TBodyDot
% d/dt(inv(Tbase) = -inv(TBase) * TBaseDot * inv(Tbase)

invTBasedot = -TBase \ TBaseDot / TBase;
ret = invTBasedot * TBody + TBase \ TBodyDot;

end