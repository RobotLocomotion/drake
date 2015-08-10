function testResolveCenterOfPressure()

testNoNormalForce();
testBackAndForth();
testRepInvariants();
testMultipleCoPs();
testMex();

end

function testNoNormalForce()
torque = randn(3, 1);
force = randn(3, 1);
normal = randn(3, 1);
normal = normal / norm(normal);
force = force - dot(normal, force) * normal;
point_on_contact_plane = randn(3, 1);
[cop, normal_torque] = resolveCenterOfPressure(torque, force, normal, point_on_contact_plane);
valuecheck(nan(3, 1), cop);
valuecheck(nan, normal_torque);
end

function testBackAndForth()
cop = randn(3, 1);
force = randn(3, 1);
normal = randn(3, 1);
normal = normal / norm(normal);
normal_torque = randn;
torque = cross(cop, force) + normal_torque * normal;
point_on_contact_plane = cop + cross(normal, randn(3, 1));
[cop_back, normal_torque_back] = resolveCenterOfPressure(torque, force, normal, point_on_contact_plane);
valuecheck(cop, cop_back, 1e-10);
valuecheck(normal_torque, normal_torque_back, 1e-10);

end

function testRepInvariants()
torque = randn(3, 1);
force = randn(3, 1);
normal = randn(3, 1);
normal = normal / norm(normal);
point_on_contact_plane = randn(3, 1);
cop = resolveCenterOfPressure(torque, force, normal, point_on_contact_plane);
valuecheck(dot(normal, point_on_contact_plane), dot(normal, cop));
end

function testMultipleCoPs()
n = 5;
torque = randn(3, n);
force = randn(3, n);
normal = randn(3, n);
for i = 1 : n
  normal(:, i) = normal(:, i) / norm(normal(:, i));
end
point_on_contact_plane = randn(3, n);
force(:, 3) = force(:, 3) - dot(normal(:, 3), force(:, 3)) * normal(:, 3);

[cop, normal_torque] = resolveCenterOfPressure(torque, force, normal, point_on_contact_plane);

for i = 1 : n
  [copi, normal_torquei] = resolveCenterOfPressure(torque(:, i), force(:, i), normal(:, i), point_on_contact_plane(:, i));
  valuecheck(copi, cop(:, i));
  valuecheck(normal_torquei, normal_torque(:, i));
end

end

function testMex()
torque = randn(3, 1);
force = randn(3, 1);
normal = randn(3, 1);
normal = normal / norm(normal);
point_on_contact_plane = randn(3, 1);
cop = resolveCenterOfPressure(torque, force, normal, point_on_contact_plane);
cop_mex = resolveCenterOfPressuremex(torque, force, normal, point_on_contact_plane);
valuecheck(cop, cop_mex);
end
