function [Adot_times_v, dAdot_times_v] = centroidalMomentumMatrixDotTimesV(robot, kinsol)
compute_gradients = nargout > 1;
nq = robot.getNumPositions();

if compute_gradients
  [inertias_world, dinertias_world] = inertiasInWorldFrame(robot, kinsol);
  [com, dcom] = robot.getCOM(kinsol);
else
  inertias_world = inertiasInWorldFrame(robot, kinsol);
  com = robot.getCOM(kinsol);
end

NB = robot.getNumBodies();
hdots = cellfun(@mtimes, inertias_world, kinsol.JdotV, 'UniformOutput', false);
if compute_gradients
  dhdots = cellfun(@matGradMultMat, inertias_world, kinsol.JdotV, dinertias_world, kinsol.dJdotVdq, 'UniformOutput', false);
end

for i = 2 : NB
  inertia_times_twist = inertias_world{i} * kinsol.twists{i};
  hdots{i} = hdots{i} + crf(kinsol.twists{i}) * inertia_times_twist;
  
  if compute_gradients
    dinertia_times_twist = inertias_world{i} * kinsol.dtwistsdq{i} + matGradMult(dinertias_world{i}, kinsol.twists{i});
    dhdots{i} = dhdots{i} + dcrf(kinsol.twists{i}, inertia_times_twist, kinsol.dtwistsdq{i}, dinertia_times_twist);
  end
end

transform_com_to_world = zeros(4) * kinsol.q(1); % for TaylorVar
transform_com_to_world(1:3, 1:3) = eye(3);
transform_com_to_world(4, 4) = 1;
transform_com_to_world(1:3, 4) = com;
AdH = transformAdjoint(transform_com_to_world);
Adot_times_v = sum(horzcat(hdots{2:end}), 2);  % Adot_times_v in world
Adot_times_v = AdH' * Adot_times_v; % change frame to CoM
% plus AdHdot * h, but this is zero because of the way the com frame is
% defined.
if compute_gradients
  dtransform_com_to_world = zeros(numel(transform_com_to_world), nq);
  dtransform_com_to_world = setSubMatrixGradient(dtransform_com_to_world, dcom, 1:3, 4, size(transform_com_to_world));
  dAdot_times_v = sum(cat(3, dhdots{2:end}), 3);
  dAdot_times_v = dTransformSpatialForce(inv(transform_com_to_world), Adot_times_v, -dtransform_com_to_world, dAdot_times_v);
end

end
