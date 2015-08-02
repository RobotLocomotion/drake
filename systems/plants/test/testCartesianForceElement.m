function testCartesianForceElement

%% test cartesian forces first on a planar model

p = PlanarRigidBodyManipulator('MassSpringDamper.urdf');
p_cartesian = compile(addForceElement(p,RigidBodyCartesianForce('thrust',p.findLinkId('mass'))));
%getInputFrame(p_cartesian)

p_thrust = PlanarRigidBodyManipulator('MassSpringDamperThrust.urdf');
%getInputFrame(p_thrust)

for i=1:20
  t = randn();
  x = randn(getNumStates(p),1);
  thrust = randn();
  valuecheck(dynamics(p_cartesian,t,x,[0;thrust;0;0]),dynamics(p_thrust,t,x,thrust));
end

%% test cartesian forces on a 3D model

p = RigidBodyManipulator('MassSpringDamper.urdf');
p_cartesian = compile(addForceElement(p,RigidBodyCartesianForce('thrust',p.findLinkId('mass'))));
%getInputFrame(p_cartesian)

p_thrust = RigidBodyManipulator('MassSpringDamperThrust.urdf');
%getInputFrame(p_thrust)

for i=1:20
  t = randn();
  x = randn(getNumStates(p),1);
  thrust = randn();
  valuecheck(dynamics(p_cartesian,t,x,[0;thrust;0;0]),dynamics(p_thrust,t,x,thrust));
end