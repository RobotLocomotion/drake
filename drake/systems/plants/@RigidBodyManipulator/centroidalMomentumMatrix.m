function [A, dA] = centroidalMomentumMatrix(robot, kinsol, robotnum, in_terms_of_qdot)
% function [A, dA] = centroidalMomentumMatrix(robot, kinsol, robotnum, in_terms_of_qdot)
% Computes the centroidal momentum matrix (CMM), i.e. the matrix that maps
% the joint velocity vector v to the robot's momentum h = [k; l], with
% angular momentum k and linear momentum l, expressed in a centroidal
% frame. See Orin13.
%
% @param kinsol solution structure obtained from doKinematics
%
% @retval A CMM
% @retval dA gradient of A with respect to coordinate vector q

if nargin < 3
  robotnum = 1;
end
if nargin < 4
  in_terms_of_qdot = false;
end

compute_gradients = nargout > 1;

if ~isstruct(kinsol)  
  % treat input as centroidalMomentumMatrix(model,q)
  q = kinsol;
  kinsol_options.compute_gradients = compute_gradients;
  kinsol = robot.doKinematics(q, double.empty(0, 1) * q(1), kinsol_options);
end

if kinsol.mex
  A = centroidalMomentumMatrixmex(robot.mex_model_ptr, kinsol.mex_ptr, robotnum - 1, in_terms_of_qdot);
  if kinsol.has_gradients
    [A, dA] = eval(A);
    nq = length(kinsol.q);
    dA = dA(:, 1 : nq);
  end
else
  if compute_gradients
    [A, dA] = worldMomentumMatrix(robot, kinsol, robotnum, in_terms_of_qdot);
  else
    A = worldMomentumMatrix(robot, kinsol, robotnum, in_terms_of_qdot);
  end
  
  com = robot.getCOM(kinsol);
  transform_com_to_world = zeros(4) * kinsol.q(1); % for TaylorVar
  transform_com_to_world(1:3, 1:3) = eye(3);
  transform_com_to_world(4, 4) = 1;
  transform_com_to_world(1:3, 4) = com;
  AdH = transformAdjoint(transform_com_to_world);
  A = AdH' * A;

  if compute_gradients
    total_mass = getMass(robot, robotnum);
    dcom = A(4:6, :) / total_mass;
    if ~in_terms_of_qdot
      qdotToV = kinsol.qdotToV(2:end);
      dcom = dcom * blkdiag(qdotToV{:});
    end
    nq = robot.getNumPositions();
    dtransform_com_to_world = zeros(numel(transform_com_to_world), nq);
    dtransform_com_to_world = setSubMatrixGradient(dtransform_com_to_world, dcom, 1:3, 4, size(transform_com_to_world));
    dA = dTransformSpatialForce(inv(transform_com_to_world), A, -dtransform_com_to_world, dA);
  end
end
end

function [A, dA] = worldMomentumMatrix(robot, kinsol, robotnum, in_terms_of_qdot)
compute_gradients = nargout > 1;
if compute_gradients
  [inertias_world, dinertias_world] = inertiasInWorldFrame(robot, kinsol);
  [crbs_world, dcrbs_world] = compositeRigidBodyInertias(robot, inertias_world, dinertias_world);
else
  inertias_world = inertiasInWorldFrame(robot, kinsol);
  crbs_world = compositeRigidBodyInertias(robot, inertias_world);
end

if in_terms_of_qdot
  ncols = robot.getNumPositions();
else
  ncols = robot.getNumVelocities();
end

size_A = [6, ncols];
A = zeros(size_A) * kinsol.q(1);
if compute_gradients
  dA = zeros(prod(size_A), robot.getNumPositions());
end

for i = 2 : length(robot.body)
  if isBodyPartOfRobot(robot, robot.body(i), robotnum)
    if in_terms_of_qdot
      IcJ = crbs_world{i} * kinsol.J{i};
      cols_joint = robot.body(i).position_num;
      A(:, cols_joint) = IcJ * kinsol.qdotToV{i};
      if compute_gradients
        dIcJ = matGradMultMat(crbs_world{i}, kinsol.J{i}, dcrbs_world{i}, kinsol.dJdq{i});
        dA = setSubMatrixGradient(dA, matGradMultMat(IcJ, kinsol.qdotToV{i}, dIcJ, kinsol.dqdotToVdq{i}), 1:6, cols_joint, size_A);
      end
    else
      cols_joint = robot.body(i).velocity_num;
      A(:, cols_joint) = crbs_world{i} * kinsol.J{i};
      if compute_gradients
        dA = setSubMatrixGradient(dA, matGradMultMat(crbs_world{i}, kinsol.J{i}, dcrbs_world{i}, kinsol.dJdq{i}), 1:6, cols_joint, size_A);
      end
    end
  end
end
end
