function testRigidBodyPassByValue
options.floating = true;
m1 = RigidBodyManipulator('FallingBrick.urdf',options);

% Extremely simple check
assert(~isa(m1.body(end),'handle'), ...
  'RigidBody should not be a handle class');

% Slightly more involved check
mass = m1.body(end).mass;
com = m1.body(end).com;
inertia = m1.body(end).inertia;

setInertial(m1.body(end), 2*mass, com, inertia);

valuecheck(m1.body(end).mass,mass);
end
