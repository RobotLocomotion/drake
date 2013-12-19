function testGeometricJacobian

testAtlas('rpy');
% testAtlas('quat'); % TODO: generating robot currently results in an error.

end

function testAtlas(floatingJointType)

options.floating = floatingJointType;
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
robot = RigidBodyManipulator(fullfile('../../../examples/Atlas/urdf/atlas_minimal_contact.urdf'),options);
warning(w);

nq = robot.getNumStates() / 2; % TODO
nv = robot.getNumStates() / 2; % TODO

nBodies = length(robot.body);

bodyRange = [1, nBodies];

nTests = 50;
testNumber = 1;
while testNumber < nTests
    q = randn(nq, 1);
    v = randn(nv, 1);
    kinsol = robot.doKinematics(q,false,false, v);

    base = randi(bodyRange);
    endEffector = randi(bodyRange);
    if base ~= endEffector
        expressedIn = base;

        [J, vIndices] = robot.geometricJacobian(kinsol, base, endEffector, expressedIn);
        twist = J * (v(vIndices));

        HBase = kinsol.T{base};
        HBody = kinsol.T{endEffector};
        HBaseDot = kinsol.Tdot{base};
        HBodyDot = kinsol.Tdot{endEffector};

        HBodyToBase = HBase \ HBody;
        transformDot = toTildeForm(twist) * HBodyToBase;
        transformDotForwardKin = computeHdotRelative(HBase, HBody, HBaseDot, HBodyDot);
        valuecheck(transformDot, transformDotForwardKin, 1e-12);
        testNumber = testNumber + 1;
    end
end
end

function ret = toTildeForm(twist)
omega = twist(1 : 3);
v = twist(4 : 6);
ret = [vectorToSkewSymmetric(omega), v;
       zeros(1, 4)];
end

function HdotRelative = computeHdotRelative(HBase, HBody, HBaseDot, HBodyDot)
% HRelative = inv(HBase) * HBody
% d/dt(HRelative) = d/dt(inv(HBase) * HBody) + inv(HBase) * HBodyDot
% d/dt(inv(Hbase) = -inv(HBase) * HBaseDot * inv(Hbase)

invHBase = inv(HBase);
invHBasedot = -invHBase * HBaseDot * invHBase;
HdotRelative = invHBasedot * HBody + invHBase * HBodyDot;

end