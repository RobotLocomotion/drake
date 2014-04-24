function [inertias, composite_inertias] = computeInertiasInWorld(manipulator, kinsol)
NB = length(manipulator.body);
inertias = cell(1, NB);
inertias{1} = zeros(6, 6);
for i = 2 : NB
  body = manipulator.body(i);
  bodyToWorld = kinsol.T{i};
  worldToBody = homogTransInv(bodyToWorld);
  AdWorldToBody = transformAdjoint(worldToBody);
  inertias{i} = AdWorldToBody' * body.I * AdWorldToBody;
end

if nargout > 1
  composite_inertias = computeCompositeRigidBodyInertias(manipulator, inertias);
end
end

function ret = computeCompositeRigidBodyInertias(manipulator, inertias_world)
% computes composite rigid body inertias expressed in world frame
NB = length(inertias_world);
ret = inertias_world;
for i = NB : -1 : 2
  body = manipulator.body(i);
  ret{body.parent} = ret{body.parent} + ret{i};
end
end