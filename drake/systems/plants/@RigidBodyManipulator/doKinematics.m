function kinsol = doKinematics(model, q, v, options, qd_old)
% kinsol=doKinematics(model, q, v, options, qd_old)
% Precomputes information necessary to compute kinematic and dynamic
% quantities such as Jacobians (@see forwardKin, @see geometricJacobian),
% mass matrix and dynamics bias (@see manipulatorDynamics), and other
% biases, e.g. Jdot * v (@see geometricJacobianDotTimesV).
% 
% @param q joint position vector. Must be model.getNumPositions() x 1
% @param v joint velocity vector. Must be model.getNumVelocities() x 1 or
% empty
% @param options options struct. Fields:
% use_mex: whether or not to use the mex implementation. @default true
% compute_gradients: boolean, if true: additionally precompute
%   gradients of all quantities in doKinematics that are precomputed when
%   compute_gradients is false. Set options.compute_gradients to true if
%   gradients of Jacobians or dynamics are required. If only Jacobians
%   (geometric or from forwardKin) are required, setting compute_gradients
%   to true is *not* required. @default false
% compute_JdotV: whether or not to precompute quantities necessary to
% compute biases such as Jdot * v and the C term in manipulatorDynamics
% @default true if v is passed in and not empty.
% kinematics_cache_ptr_to_use: DrakeMexPointer. Tells doKinematics to use a
% specific kinematics cache  DrakeMexPointer. This option can be useful when
% multiple kinsols need to be valid at the same time. By default, the
% default kinematics cache pointers stored in the RigidBodyManipulator are
% used, meaning that subsequent doKinematics calls overwrite the previous
% cache, and hence invalidate kinsols from earlier doKinematics calls.
%
% @retval kinsol a structure containing the precomputed information
% (non-mex case) or a certificate of having precomputed this information
% (mex case)
%
% Note: the contents of kinsol may change with implementation details - do
% not attempt to use kinsol directly (our contract is simply that it can
% always be used by other kinematics and dynamics methods in Drake.
%
% Note: old method signature: 
% kinsol = doKinematics(model,q,b_compute_second_derivatives,use_mex,qd)
% This method signature is deprecated, please transition to using the
% options struct. Using the old signature is supported for now.

checkDirty(model);

% method signature transition
if nargin > 3
  options_copy = options;
end
warn_signature_changed = false;
if nargin == 5
  warn_signature_changed = true;
  options = struct();
  options.compute_gradients = v;
  options.use_mex = options_copy;
  v = qd_old;
elseif nargin == 4
  if islogical(options)
    warn_signature_changed = true;
    options = struct();
    options.compute_gradients = v;
    options.use_mex = options_copy;
    v = [];
  end
elseif nargin == 3
  options = struct();
  if islogical(v)
    warn_signature_changed = true;
    options.compute_gradients = v;
    v = [];
  end
else
  options = struct();
  v = [];
end

if ~isfield(options, 'use_mex'), options.use_mex = true; end
if ~isfield(options, 'compute_gradients'), options.compute_gradients = false; end
if ~isfield(options, 'compute_JdotV'), options.compute_JdotV = ~isempty(v); end
if ~isfield(options, 'force_new_kinsol'), options.force_new_kinsol = false; end
if ~isfield(options, 'kinematics_cache_ptr_to_use'), options.kinematics_cache_ptr_to_use = []; end

if warn_signature_changed
  % TODO: turn on warning
%   model.warning_manager.warnOnce('Drake:RigidBodyManipulator:doKinematics:method_signature_changed', ...
%     'Called doKinematics using arguments corresponding to the old method signature. This will be phased out; please update your call to match the new signature.');
end

% TODO: currently lots of branch-induced sparsity unexploited in
% gradient computations.
kinsol.q = q;
kinsol.v = v;

if (options.use_mex && model.mex_model_ptr~=0 && isnumeric(q))
  if isempty(options.kinematics_cache_ptr_to_use)
    if options.compute_gradients
      kinsol.mex_ptr = model.default_kinematics_cache_ptr_with_gradients;
    else
      kinsol.mex_ptr = model.default_kinematics_cache_ptr_no_gradients;
    end
  else
    kinsol.mex_ptr = options.kinematics_cache_ptr_to_use;
  end
  
  doKinematicsmex(model.mex_model_ptr, kinsol.mex_ptr, q, v, options.compute_JdotV);
  kinsol.mex = true;
else
  kinsol.mex = false;
  
  bodies = model.body;
  kinsol.T = computeTransforms(bodies, q);
  if options.compute_gradients
    [kinsol.S, kinsol.dSdq] = computeMotionSubspaces(bodies, q);
  else
    kinsol.S = computeMotionSubspaces(bodies, q);
  end
  kinsol.J = computeJ(kinsol.T, kinsol.S);
  
  if options.compute_gradients
    [kinsol.qdotToV, kinsol.dqdotToVdqi, kinsol.dqdotToVdq] = computeQdotToV(bodies, q);
    [kinsol.vToqdot, kinsol.dvToqdotdqi, kinsol.dvToqdotdq] = computeVToqdot(bodies, q);
  else
    kinsol.qdotToV = computeQdotToV(bodies, q);
    kinsol.vToqdot = computeVToqdot(bodies, q);
  end
  if ~isempty(v)
    kinsol.qd = computeQdot(bodies, kinsol.vToqdot, v, length(q));
  end
  
  if options.compute_gradients
    kinsol.dTdq = computeTransformGradients(bodies, kinsol.T, kinsol.S, kinsol.qdotToV, length(q));
    kinsol.dJdq = computedJdq(bodies, kinsol.T, kinsol.S, kinsol.dTdq, kinsol.dSdq);
  end
  
  if ~isempty(v)
    if options.compute_gradients
      [kinsol.twists, kinsol.dtwistsdq] = computeTwistsInBaseFrame(bodies, kinsol.J, v, kinsol.dJdq);
      if options.compute_JdotV
        [SdotV, dSdotVdq, dSdotVdv] = computeMotionSubspacesDotV(bodies, q, v);
        [kinsol.JdotV, kinsol.dJdotVdq, kinsol.dJdotVidv] = computeJacobianDotV(model, kinsol, SdotV, dSdotVdq, dSdotVdv);
      end
    else
      kinsol.twists = computeTwistsInBaseFrame(bodies, kinsol.J, v);
      if options.compute_JdotV
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
    q_body = q(body.position_num); % qi is 6x1
    T_body_to_parent = body.Ttree*jointTransform(body, q_body);
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
  q_body = q(body.position_num);
  if compute_gradients
    [S{i}, dSdq{i}] = motionSubspace(body, q_body);
  else
    S{i} = motionSubspace(body, q_body);
  end
end
end

function ret = computeTransformGradients(bodies, T, S, qdotToV, nq)
% computes the gradients of T{i} with respect to q
% makes use of the fact that the gradients of the joint transforms are
% dT/dq = dTdot/dqdot, where Tdot depends on v through the joint motion
% subspace S and qdot depends on v via the qdotToV mapping.

nb = length(bodies);
ret = cell(1, nb);
ret{1} = zeros(16, nq);
for i = 2 : nb
  body = bodies(i);
  T_body_to_parent = T{body.parent} \ T{i};
  
  dT_body_to_parentdqi = dHomogTrans(T_body_to_parent, S{i}, qdotToV{i});
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
  dSidq = zeros(numel(Si), nq) * dTdq{i}(1); % to make TaylorVar work better
  dSidq(:, body.position_num) = dSdq{i};
  ret{i} = dTransformSpatialMotion(T{i}, Si, dTdq{i}, dSidq);
end
end

function [Vq, dVqdqi, dVqdq] = computeQdotToV(bodies, q)
compute_gradient = nargout > 1;
nb = length(bodies);
Vq = cell(1, nb);
if compute_gradient
  dVqdqi = cell(1, nb);
  dVqdq = cell(1, nb);
end
nq = length(q);

for i = 2 : nb
  body = bodies(i);
  q_body = q(body.position_num);
  if compute_gradient
    [Vq{i}, dVqdqi{i}] = jointQdot2v(body, q_body);
    dVqdq{i} = zeros(numel(Vq{i}), nq);
    dVqdq{i}(:, body.position_num) = dVqdqi{i};
  else
    Vq{i} = jointQdot2v(body, q_body);
  end
end
end

function [VqInv, dVqInvdqi, dVqInvdq] = computeVToqdot(bodies, q)
compute_gradient = nargout > 1;
nb = length(bodies);
VqInv = cell(1, nb);
if compute_gradient
  dVqInvdqi = cell(1, nb);
  dVqInvdq = cell(1, nb);
end
nq = length(q);

for i = 2 : nb
  body = bodies(i);
  q_body = q(body.position_num);
  if compute_gradient
    [VqInv{i}, dVqInvdqi{i}] = jointV2qdot(body, q_body);
    dVqInvdq{i} = zeros(numel(VqInv{i}), nq);
    dVqInvdq{i}(:, body.position_num) = dVqInvdqi{i};
  else
    VqInv{i} = jointV2qdot(body, q_body);
  end
end
end

function qd = computeQdot(bodies, vToqdot, v, nq)
% see http://undocumentedmatlab.com/blog/trapping-warnings-efficiently
% this turns a warning with the specified ID into an error.
original_warning_state = warning('error', 'Drake:TaylorVar:DoubleConversion');
qd = zeros(nq, 1);
try
  for i = 2 : length(bodies)
    body = bodies(i);
    qd(body.position_num) = vToqdot{i} * v(body.velocity_num);
  end
catch
  % do it the inefficient but TaylorVar-proof way
  qd = blkdiag(vToqdot{:}) * v;
end
warning(original_warning_state);
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
      + dTransformSpatialMotion(kinsol.T{i}, SdotVi{i}, kinsol.dTdq{i}, dSdotVidq);
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