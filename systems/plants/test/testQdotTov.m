function testQdotTov
testAtlas('rpy');
testAtlas('quat');
end

function testAtlas(floatingType)
m = createAtlas(floatingType);
nv = m.getNumVelocities();
q = getRandomConfiguration(m);
kinsol = doKinematics(m, q, false, false);
valuecheck(eye(nv), kinsol.qdotToV * kinsol.vToqdot, 1e-12);
end