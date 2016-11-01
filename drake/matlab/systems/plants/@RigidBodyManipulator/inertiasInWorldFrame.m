function [inertias, dinertias] = inertiasInWorldFrame(manipulator, kinsol)
% Transforms 6x6 inertia matrices from body frame to world frame
%
% @param kinsol solution structure from doKinematics
% 
% @retval inertias inertia matrices in world frame
% @retval dinertias gradients of inertia matrices with respect to joint
% configuration vector q

compute_gradients = nargout > 1;

NB = length(manipulator.body);
nq = manipulator.getNumPositions();

inertias = cell(1, NB);
inertias{1} = zeros(6, 6);

if compute_gradients
  dinertias = cell(1, NB);
  dinertias{1} = zeros(numel(inertias{1}), nq);
end

for i = 2 : NB
  body = manipulator.body(i);
  body_to_world = kinsol.T{i};
  world_to_body = homogTransInv(body_to_world);
  Ad_world_to_body = transformAdjoint(world_to_body);
  inertias{i} = Ad_world_to_body' * body.I * Ad_world_to_body;
  
  if compute_gradients
    dbody_to_world = kinsol.dTdq{i};
    dbody_to_world = dHomogTransInv(body_to_world, dbody_to_world);
    dAd_world_to_body = dTransformSpatialMotion(world_to_body, eye(6), dbody_to_world, zeros(numel(Ad_world_to_body), nq));
    dinertias{i} = quadFormGrad(Ad_world_to_body, body.I, dAd_world_to_body);
  end
end
end