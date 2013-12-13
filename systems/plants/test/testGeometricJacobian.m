function testGeometricJacobian

robot = RigidBodyManipulator('../../../examples/FurutaPendulum/FurutaPendulum.urdf');

if (0) % need to resolve floating base issue
options.floating = true;
w = warning('off','Drake:RigidBodyManipulator:UnsupportedVelocityLimits');
warning('off','Drake:RigidBodyManipulator:UnsupportedContactPoints');
warning('off','Drake:RigidBodyManipulator:UnsupportedJointLimits');
robot = RigidBodyManipulator(fullfile('../../../examples/Atlas/urdf/atlas_minimal_contact.urdf'),options);
warning(w);
end

nq = robot.getNumStates() / 2; % TODO
nv = robot.getNumStates() / 2; % TODO

nBodies = length(robot.body);

% currently need to start at 2 (i.e. exclude the 6-DoF joint), because the rotational 
% part of itsvelocity is parameterized as (yawd, pitchd, rolld) or 
% quatd, whereas geometricJacobian expects omega
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