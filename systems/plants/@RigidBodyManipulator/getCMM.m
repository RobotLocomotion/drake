function [A, Adot_times_v, dA, dAdot_times_v] = getCMM(robot, kinsol)

compute_Adot_times_v = nargout > 1;
compute_gradients = nargout > 2;

if compute_gradients
  [inertias_world, dinertias_world] = inertiasInWorldFrame(robot, kinsol);
  [crbs_world, dcrbs_world] = compositeRigidBodyInertias(robot, inertias_world, dinertias_world);
  [com, dcom] = robot.getCOM(kinsol);
else
  inertias_world = inertiasInWorldFrame(robot, kinsol);
  crbs_world = compositeRigidBodyInertias(robot, inertias_world);
  com = robot.getCOM(kinsol);
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
  dA = dAdHTransposeTimesX(transform_com_to_world, A, dtransform_com_to_world, dA);
end

if compute_Adot_times_v
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
  
  Adot_times_v = sum(horzcat(hdots{2:end}), 2);  % Adot_times_v in world
  Adot_times_v = AdH' * Adot_times_v; % change frame to CoM
  % plus AdHdot * h, but this is zero because of the way the com frame is
  % defined.
  if compute_gradients
    dAdot_times_v = sum(cat(3, dhdots{2:end}), 3);
    dAdot_times_v = dAdHTransposeTimesX(transform_com_to_world, Adot_times_v, dtransform_com_to_world, dAdot_times_v);
  end
end
end