function ret = compositeRigidBodyInertias(manipulator, inertias_world)
% computes composite rigid body inertias expressed in world frame
NB = length(inertias_world);
ret = inertias_world;
for i = NB : -1 : 2
  body = manipulator.body(i);
  ret{body.parent} = ret{body.parent} + ret{i};
end
end