function [A, Adot_times_v] = getCMM(robot, kinsol)
inertias_world = inertiasInWorldFrame(robot, kinsol);
crbs_world = compositeRigidBodyInertias(robot, inertias_world);
ABlocks = cellfun(@mtimes, crbs_world(2:end), kinsol.J(2:end), 'UniformOutput', false);
A = [ABlocks{:}]; % world momentum matrix
% A = cell2mat(ABlocks); % doesn't work with TaylorVar

com = robot.getCOM(kinsol.q);
transform_com_to_world = eye(4);
transform_com_to_world(1:3, 4) = com;
AdH = transformAdjoint(transform_com_to_world);
A = AdH' * A;

if nargout > 1
  NB = robot.getNumBodies();
  hdots = cellfun(@mtimes, inertias_world, kinsol.JdotV, 'UniformOutput', false);
  for i = 2 : NB
    hdots{i} = hdots{i} + crf(kinsol.twists{i}) * inertias_world{i} * kinsol.twists{i};
  end
  Adot_times_v = sum(cell2mat(hdots), 2); % Adot_times_v in world
  Adot_times_v = AdH' * Adot_times_v; % change frame to CoM
  % plus AdHdot * h, but this is zero because of the way the com frame is
  % defined.
end
end