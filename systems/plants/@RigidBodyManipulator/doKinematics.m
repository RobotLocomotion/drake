function kinsol = doKinematics(model, q, compute_gradients, use_mex, v, compute_JdotV)
% kinsol=doKinematics(model, q, compute_gradients, use_mex, v, compute_JdotV)
% Computes the (forward) kinematics of the manipulator.
%
% @retval kinsol a certificate containing the solution (or information
% about the solution) that must be passed to model.forwardKin() in order to
% be evaluated.  Note: the contents of kinsol may change with
% implementation details - do not attempt to use kinsol directly (our
% contract is simply that it can always be passed to forwardKin to retrieve
% the answer).
%

% TODO: currently lots of branch-induced sparsity unexploited in
% gradient computations.

checkDirty(model);
if nargin<5, v=[]; end
if nargin<6 && ~isempty(v), compute_JdotV=true; end
if nargin<4, use_mex = true; end
if nargin<3, compute_gradients = false; end

% use_mex = false; % for now

kinsol.q = q;
kinsol.v = v;

if (use_mex && model.mex_model_ptr~=0 && isnumeric(q))
  doKinematicsmex(model.mex_model_ptr,q,compute_gradients,v);
  kinsol.mex = true;
else
  kinsol.mex = false;
  
  bodies = model.body;
  kinsol.T = computeTransforms(bodies, q);
  if compute_gradients
    [S, dSdq] = computeMotionSubspaces(bodies, q);
  else
    S = computeMotionSubspaces(bodies, q);
  end
  kinsol.J = computeJ(kinsol.T, S);
  
  if compute_gradients
    [kinsol.qdotToV, kinsol.dqdotToVdq] = qdotToV(model, q);
    [kinsol.vToqdot, kinsol.dvToqdotdq] = vToqdot(model, q);
  else
    kinsol.qdotToV = qdotToV(model, q);
    kinsol.vToqdot = vToqdot(model, q);
  end

  if compute_gradients
    kinsol.dTdq = computeTransformGradients(bodies, kinsol.T, S, kinsol.qdotToV);
    kinsol.dJdq = computedJdq(bodies, kinsol.T, S, kinsol.dTdq, dSdq);
  end
  
  if ~isempty(v)
    if compute_gradients
      [kinsol.twists, kinsol.dtwistsdq] = computeTwistsInBaseFrame(bodies, kinsol.J, v, kinsol.dJdq);
      if compute_JdotV
        [SdotV, dSdotVdq, dSdotVdv] = computeMotionSubspacesDotV(bodies, q, v);
        [kinsol.JdotV, kinsol.dJdotVdq, kinsol.dJdotVidv] = computeJacobianDotV(model, kinsol, SdotV, dSdotVdq, dSdotVdv);
      end
    else
      kinsol.twists = computeTwistsInBaseFrame(bodies, kinsol.J, v);
      if compute_JdotV
        SdotV = computeMotionSubspacesDotV(bodies, q, v);
        kinsol.JdotV = computeJacobianDotV(model, kinsol, SdotV);
      end
    end
  end
end
end


function T = computeTransforms(bodies, q)
nb = length(bodies);
T = cell(1, nb);
for i=1:nb
  body = bodies(i);
  if body.parent<1
    T{i} = body.Ttree;
  else
    if body.floating==1
      qi = q(body.position_num); % qi is 6x1
      rx = rotx(qi(4)); ry = roty(qi(5)); rz = rotz(qi(6));
      TJ = [rz*ry*rx,qi(1:3);zeros(1,3),1];
    elseif body.floating==2
      qi = q(body.position_num);  % qi is 7x1
      TJ = [quat2rotmat(qi(4:7)),qi(1:3);zeros(1,3),1];
    else
      qi = q(body.position_num);
      TJ = Tjcalc(body.pitch,qi);
    end
    T_body_to_parent = body.Ttree*homogTransInv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
    T{i}=T{body.parent}*T_body_to_parent;
  end
end
end

function [S, dSdq] = computeMotionSubspaces(bodies, q)
compute_gradients = nargout > 1;
nb = length(bodies);
S = cell(1, nb);
if compute_gradients
  dSdq = cell(1, nb);
end

for i = 2 : nb
  body = bodies(i);
  qBody = q(body.position_num);
  if compute_gradients
    [S{i}, dSdq{i}] = motionSubspace(body, qBody);
  else
    S{i} = motionSubspace(body, qBody);
  end
end
end

function ret = computeTransformGradients(bodies, T, S, qdotToV)
% computes the gradients of T{i} with respect to q
% makes use of the fact that the gradients of the joint transforms are
% dT/dq = dTdot/dqdot, where Tdot depends on v through the joint motion
% subspace S and qdot depends on v via the qdotToV mapping.

nb = length(bodies);
nq = size(qdotToV, 2);
ret = cell(1, nb);
ret{1} = zeros(16, nq);
for i = 2 : nb
  body = bodies(i);
  T_body_to_parent = T{body.parent} \ T{i};
  qdotToVi = qdotToV(body.velocity_num, body.position_num);
  
  dT_body_to_parentdqi = dHomogTrans(T_body_to_parent, S{i}, qdotToVi);
  dT_body_to_parentdq = zeros(numel(T{i}), nq) * dT_body_to_parentdqi(1); % to make TaylorVar work better
  dT_body_to_parentdq(:, body.position_num) = dT_body_to_parentdqi;
  ret{i} = matGradMultMat(...
    T{body.parent}, T_body_to_parent, ret{body.parent}, dT_body_to_parentdq);
end
end

function J = computeJ(T, S)
% Computes motion subspaces transformed to world frame, i.e.
% transformAdjoint(T{i}) * S{i}

nb = length(T);
J = cell(1, nb);
for i = 2 : nb
  J{i} = transformAdjoint(T{i}) * S{i};
end
end

function ret = computedJdq(bodies, T, S, dTdq, dSdq)
% Computes the gradient of transformAdjoint(T{i}) * S{i} with respect to q
% For uses, see e.g. geometricJacobianDotTimesV, gradients of mass matrix
% and bias vector in manipulatorDynamics

nb = length(T);
ret = cell(1, nb);
for i = 2 : nb
  body = bodies(i);
  nq = size(dTdq{i}, 2);
  Si = S{i};
  dSdqi = zeros(numel(Si), nq) * dTdq{i}(1); % to make TaylorVar work better
  dSdqi(:, body.position_num) = dSdq{i};
  ret{i} = dTransformAdjoint(T{i}, Si, dTdq{i}, dSdqi);
end
end

function [twists, dtwistsdq] = computeTwistsInBaseFrame(bodies, J, v, dJdq)
compute_gradient = nargout > 1;
if compute_gradient
  if nargin < 4
    error('must provide dJdq to compute gradient');
  end
  nq = size(dJdq{end}, 2);
end

nb = length(bodies);
twistSize = 6;

twists = cell(1, nb);
twists{1} = zeros(twistSize, 1);

if compute_gradient
  dtwistsdq = cell(1, nb);
  dtwistsdq{1} = zeros(numel(twists{1}), nq);
end

for i = 2 : nb
  body = bodies(i);
  vBody = v(body.velocity_num);

  parentTwist = twists{body.parent};
  jointTwist = J{i} * vBody;
  twists{i} = parentTwist + jointTwist;
  
  if compute_gradient
    dparentTwist = dtwistsdq{body.parent};
    dJointTwist = matGradMult(dJdq{i}, vBody);
    dtwistsdq{i} = dparentTwist + dJointTwist;
  end
end
end

function [SdotV, dSdotVdq, dSdotVdv] = computeMotionSubspacesDotV(bodies, q, v)
compute_gradients = nargout > 1;

nb = length(bodies);
SdotV = cell(1, nb);
if compute_gradients
  dSdotVdq = cell(1, nb);
  dSdotVdv = cell(1, nb);
end

for i = 2 : nb
  body = bodies(i);
  qi = q(body.position_num);
  vi = v(body.velocity_num);
  if compute_gradients
    [SdotV{i}, dSdotVdq{i}, dSdotVdv{i}] = motionSubspaceDotTimesV(body, qi, vi);
  else
    SdotV{i} = motionSubspaceDotTimesV(body, qi, vi);
  end
end
end

function [JdotV, dJdotVdq, dJdotVidv] = computeJacobianDotV(model, kinsol, SdotVi, dSidotVidqi, Sdot)
% bodies, T, twists, SdotV, J, dTdq, dtwistsdq, S, dSdotVdq, Sdot, v
compute_gradients = nargout > 1;

bodies = model.body;
world = 1;
nb = length(bodies);
if compute_gradients
  nq = size(kinsol.dTdq{end}, 2);
  nv = length(kinsol.v);
end

JdotV = cell(1, nb);
JdotV{1} = zeros(6, 1);
if compute_gradients
  dJdotVdq = cell(1, nb);
  dJdotVdq{1} = zeros(numel(JdotV{1}), nq);
  dJdotVidv = cell(1, nb);
  dJdotVidv{1} = zeros(6, nv);
end

for i = 2 : nb
  body = bodies(i);
  parent = body.parent;

  % joint_accel = transformSpatialAcceleration(kinsol, parent, i, i, world, SdotV{i});
  % faster in this case, and easier to derive gradients:
  twist = kinsol.twists{i};
  joint_twist = kinsol.twists{i} - kinsol.twists{parent};
  joint_accel = crm(twist) * joint_twist + transformAdjoint(kinsol.T{i}) * SdotVi{i};
  JdotV{i} = JdotV{parent} + joint_accel;
  
  if compute_gradients
    % q gradient
    dtwistdq = kinsol.dtwistsdq{i};
    djoint_twistdq = kinsol.dtwistsdq{i} - kinsol.dtwistsdq{parent};
    dSdotVidq = zeros(numel(SdotVi{i}), nq);
    dSdotVidq(:, body.position_num) = dSidotVidqi{i};
    djoint_acceldq = dcrm(twist, joint_twist, dtwistdq, djoint_twistdq)...
      + dTransformAdjoint(kinsol.T{i}, SdotVi{i}, kinsol.dTdq{i}, dSdotVidq);
    dJdotVdq{i} = dJdotVdq{parent} + djoint_acceldq;

    % v gradient: dJdot_times_vi/dv
    [dtwistdv, v_indices] = geometricJacobian(model, kinsol, world, i, world);
%     dtwistdv = zeros(6, nv);
%     dtwistdv(:, v_indices) = J;
    nv_branch = length(v_indices);
    body_branch_velocity_num = nv_branch - length(body.velocity_num) + 1 : nv_branch;
    djoint_twistdv = zeros(6, nv_branch);
    djoint_twistdv(:, body_branch_velocity_num) = kinsol.J{i};
    dSdotVidv = zeros(6, nv_branch);
    dSdotVidv(:, body_branch_velocity_num) = Sdot{i};
    djoint_acceldv = dcrm(twist, joint_twist, dtwistdv, djoint_twistdv)...
      + transformAdjoint(kinsol.T{i}) * dSdotVidv;
    dJdotVidv{i} = zeros(6, nv);
    dJdotVidv{i}(:, v_indices) = djoint_acceldv;
    dJdotVidv{i} = dJdotVidv{parent} + dJdotVidv{i};
  end
end
end