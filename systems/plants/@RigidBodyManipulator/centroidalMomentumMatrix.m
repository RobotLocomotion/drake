function [A, dA] = centroidalMomentumMatrix(robot, kinsol)

compute_gradients = nargout > 1;

if compute_gradients
  [inertias_world, dinertias_world] = inertiasInWorldFrame(robot, kinsol);
  [crbs_world, dcrbs_world] = compositeRigidBodyInertias(robot, inertias_world, dinertias_world);
  [com, dcom] = robot.centerOfMass(kinsol);
else
  inertias_world = inertiasInWorldFrame(robot, kinsol);
  crbs_world = compositeRigidBodyInertias(robot, inertias_world);
  com = robot.centerOfMass(kinsol);
end

ABlocks = cellfun(@mtimes, crbs_world(2:end), kinsol.J(2:end), 'UniformOutput', false);
A = [ABlocks{:}]; % 'world momentum matrix'
% A = cell2mat(ABlocks); % doesn't work with TaylorVar
transform_com_to_world = zeros(4) * kinsol.q(1); % for TaylorVar
transform_com_to_world(1:3, 1:3) = eye(3);
transform_com_to_world(4, 4) = 1;
transform_com_to_world(1:3, 4) = com;
AdH = transformAdjoint(transform_com_to_world);
A = AdH' * A;

if compute_gradients
  nq = robot.getNumPositions();
  dABlocks = cellfun(@matGradMultMat, crbs_world(2:end), kinsol.J(2:end), dcrbs_world(2:end), kinsol.dJdq(2:end), 'UniformOutput', false);
  dA = vertcat(dABlocks{:});
  dtransform_com_to_world = zeros(numel(transform_com_to_world), nq);
  dtransform_com_to_world = setSubMatrixGradient(dtransform_com_to_world, dcom, 1:3, 4, size(transform_com_to_world));
  dA = dTransformAdjointTranspose(transform_com_to_world, A, dtransform_com_to_world, dA);
end

end