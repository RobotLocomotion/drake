function testGeometricJacobian

testAtlas('rpy');
% testAtlas('quat'); % TODO: generating robot currently results in an error.

end

function testAtlas(floatingJointType)

robot = createAtlas(floatingJointType);

nq = robot.getNumPositions();
nv = robot.getNumVelocities();

nBodies = length(robot.body);

bodyRange = [1, nBodies];

nTests = 50;
testNumber = 1;
while testNumber < nTests
  q = randn(nq, 1);
  v = randn(nv, 1);
  kinsol = robot.doKinematics(q,false,false, v);
  kinsolmex = robot.doKinematics(q, false, true, v);
  
  base = randi(bodyRange);
  endEffector = randi(bodyRange);
  if base ~= endEffector
    expressedIn = base;
    
    [J, v_indices] = robot.geometricJacobian(kinsol, base, endEffector, expressedIn);
    [Jmex, v_indicesmex] = robot.geometricJacobian(kinsolmex, base, endEffector, expressedIn);
    
    valuecheck(J, Jmex);
    valuecheck(v_indices, v_indicesmex);
    
    twist = J * (v(v_indices));
    
    HBase = kinsol.T{base};
    HBody = kinsol.T{endEffector};
    HBaseDot = kinsol.Tdot{base};
    HBodyDot = kinsol.Tdot{endEffector};
    
    HBodyToBase = HBase \ HBody;
    transformDot = twistToTildeForm(twist) * HBodyToBase;
    transformDotForwardKin = relativeTdot(HBase, HBody, HBaseDot, HBodyDot);
    valuecheck(transformDot, transformDotForwardKin, 1e-12);
    testNumber = testNumber + 1;
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