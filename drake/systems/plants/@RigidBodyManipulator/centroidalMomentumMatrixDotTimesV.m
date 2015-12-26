function [Adot_times_v, dAdot_times_v] = centroidalMomentumMatrixDotTimesV(obj, kinsol, robotnum)

if nargin < 3
  robotnum = 1;
end

compute_gradients = nargout > 1;

if (kinsol.mex)
  if (obj.mex_model_ptr==0)
    error('Drake:RigidBodyManipulator:InvalidKinematics','This kinsol is no longer valid because the mex model ptr has been deleted.');
  end
  Adot_times_v = centroidalMomentumMatrixDotTimesVmex(obj.mex_model_ptr, kinsol.mex_ptr, robotnum - 1);
  if kinsol.has_gradients
    [Adot_times_v, dAdot_times_v] = eval(Adot_times_v);
    nq = length(kinsol.q);
    dAdot_times_v = dAdot_times_v(:, 1 : nq); % for backwards compatibility
  end
else
  nq = obj.getNumPositions();
  if compute_gradients
    [Adot_times_v, dAdot_times_v] = worldMomentumMatrixDotTimesV(obj, kinsol, robotnum);
  else
    Adot_times_v = worldMomentumMatrixDotTimesV(obj, kinsol, robotnum);
  end
  
  if compute_gradients
    [com, dcom] = obj.getCOM(kinsol);
  else
    com = obj.getCOM(kinsol);
  end
  
  transform_com_to_world = zeros(4) * kinsol.q(1); % for TaylorVar
  transform_com_to_world(1:3, 1:3) = eye(3);
  transform_com_to_world(4, 4) = 1;
  transform_com_to_world(1:3, 4) = com;
  AdH = transformAdjoint(transform_com_to_world);
  Adot_times_v = AdH' * Adot_times_v; % change frame to CoM
  % plus AdHdot * h, but this is zero because of the way the com frame is
  % defined.
  if compute_gradients
    dtransform_com_to_world = zeros(numel(transform_com_to_world), nq);
    dtransform_com_to_world = setSubMatrixGradient(dtransform_com_to_world, dcom, 1:3, 4, size(transform_com_to_world));
    dAdot_times_v = dTransformSpatialForce(inv(transform_com_to_world), Adot_times_v, -dtransform_com_to_world, dAdot_times_v);
  end
end
end

function [Adot_times_v, dAdot_times_v] = worldMomentumMatrixDotTimesV(obj, kinsol, robotnum)
compute_gradients = nargout > 1;

if compute_gradients
  [inertias_world, dinertias_world] = inertiasInWorldFrame(obj, kinsol);
else
  inertias_world = inertiasInWorldFrame(obj, kinsol);
end

Adot_times_v = zeros(6, 1);
if compute_gradients
  dAdot_times_v = zeros(6, obj.getNumPositions());
end
for i = 2 : obj.getNumBodies()
  if isBodyPartOfRobot(obj, obj.body(i), robotnum)
    Adot_times_v = Adot_times_v + inertias_world{i} * kinsol.JdotV{i};
    inertia_times_twist = inertias_world{i} * kinsol.twists{i};
    Adot_times_v = Adot_times_v + crf(kinsol.twists{i}) * inertia_times_twist;
    if compute_gradients
      dAdot_times_v = dAdot_times_v + matGradMultMat(inertias_world{i}, kinsol.JdotV{i}, dinertias_world{i}, kinsol.dJdotVdq{i});
      dinertia_times_twist = matGradMultMat(inertias_world{i}, kinsol.twists{i}, dinertias_world{i}, kinsol.dtwistsdq{i});
      dAdot_times_v = dAdot_times_v + dcrf(kinsol.twists{i}, inertia_times_twist, kinsol.dtwistsdq{i}, dinertia_times_twist);
    end
  end
end
end
