function inertias = inertiasInWorldFrame(manipulator, kinsol)
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
end