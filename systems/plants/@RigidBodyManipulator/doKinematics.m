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

checkDirty(model);
if nargin<6, compute_JdotV=false; end
if nargin<5, v=[]; end
if nargin<4, use_mex = true; end
if nargin<3, compute_gradients = false; end

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
  kinsol.qdotToV = qdotToV(model, q);
  kinsol.vToqdot = vToqdot(model, q);
  if compute_gradients
    kinsol.dTdq = computeTransformGradients(bodies, kinsol.T, S, kinsol.qdotToV);
    kinsol.dJdq = computedJdq(bodies, kinsol.T, S, kinsol.dTdq, dSdq);
  end
  
  if ~isempty(v)
    if compute_gradients
      [kinsol.twists, kinsol.dtwistsdq] = computeTwistsInBaseFrame(bodies, kinsol.T, S, v, kinsol.dTdq, dSdq);
      if compute_JdotV
        [SdotV, dSdotVdq] = computeMotionSubspacesDotV(bodies, q, v);
        [kinsol.JdotV, kinsol.dJdotVdq] = computeJacobianDotV(bodies, kinsol.T, kinsol.twists, SdotV, kinsol.dTdq, kinsol.dtwistsdq, dSdotVdq);
      end
    else
      kinsol.twists = computeTwistsInBaseFrame(bodies, kinsol.T, S, v);
      if compute_JdotV
        SdotV = computeMotionSubspacesDotV(bodies, q, v);
        kinsol.JdotV = computeJacobianDotV(bodies, kinsol.T, kinsol.twists, SdotV);
      end
    end
    % TODO: remove once there are no more references to this:
    kinsol.Tdot = computeTdots(kinsol.T, kinsol.twists);
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
  elseif body.floating==1
    qi = q(body.position_num); % qi is 6x1
    rx = rotx(qi(4)); ry = roty(qi(5)); rz = rotz(qi(6));
    TJ = [rz*ry*rx,qi(1:3);zeros(1,3),1];
    T{i}=T{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
  elseif body.floating==2
    qi = q(body.position_num);  % qi is 7x1
    TJ = [quat2rotmat(qi(4:7)),qi(1:3);zeros(1,3),1];
    T{i}=T{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
  else
    qi = q(body.position_num);
    
    TJ = Tjcalc(body.pitch,qi);
    T{i}=T{body.parent}*body.Ttree*inv(body.T_body_to_joint)*TJ*body.T_body_to_joint;
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

function ret = computeTransformGradients(bodies, H, S, qdotToV)
nb = length(bodies);
nq = size(qdotToV, 2);
ret = cell(1, nb);
ret{1} = zeros(16, nq);
for i = 2 : nb
  body = bodies(i);
  HToParent = H{body.parent} \ H{i};
  qdotToVi = qdotToV(body.velocity_num, body.position_num);
  dHToParentdqi = dHdq(HToParent, S{i}, qdotToVi);
  dHToParentdq = zeros(numel(H{i}), nq) * dHToParentdqi(1); % to make TaylorVar work better
  dHToParentdq(:, body.position_num) = dHToParentdqi;
  ret{i} = matGradMultMat(...
    H{body.parent}, HToParent, ret{body.parent}, dHToParentdq);
end
end

function J = computeJ(transforms, S)
nb = length(transforms);
J = cell(1, nb);
for i = 2 : nb
  H = transforms{i};
  J{i} = transformAdjoint(H) * S{i};
end
end

function ret = computedJdq(bodies, H, S, dHdq, dSdq)
nb = length(H);
ret = cell(1, nb);
for i = 2 : nb
  body = bodies(i);
  nq = size(dHdq{i}, 2);
  Si = S{i};
  dSdqi = zeros(numel(Si), nq) * dHdq{i}(1); % to make TaylorVar work better
  dSdqi(:, body.position_num) = dSdq{i};
  ret{i} = dAdHTimesX(H{i}, Si, dHdq{i}, dSdqi);
end
end

function [twists, dtwistsdq] = computeTwistsInBaseFrame(bodies, T, S, v, dTdq, dSdq)
% TODO: consider computing this based on kinsol.J, kinsol.dJdq instead

compute_gradient = nargout > 1;
if compute_gradient
  if nargin < 6
    error('must provide dTdq, dSdq to compute gradient');
  end
  nq = size(dTdq{end}, 2);
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
  jointTwist = S{i} * vBody;
  twists{i} = parentTwist + transformTwists(T{i}, jointTwist);
  
  if compute_gradient
    dparentTwist = dtwistsdq{body.parent};
    dJointTwistdq = zeros(twistSize, nq);
    dJointTwistdq(:, body.position_num) = matGradMult(dSdq{i}, vBody);
    dtwistsdq{i} = dparentTwist + dAdHTimesX(T{i}, jointTwist, dTdq{i}, dJointTwistdq);
  end
end
end

function Tdot = computeTdots(T, twist)
Tdot = cell(length(T), 1);
for i = 1 : length(T)
  Tdot{i} = twistToTildeForm(twist{i}) * T{i};
end
end

function [SdotV, dSdotVdq] = computeMotionSubspacesDotV(bodies, q, v)
compute_gradients = nargout > 1;

nb = length(bodies);
SdotV = cell(1, nb);
if compute_gradients
  dSdotVdq = cell(1, nb);
end

for i = 2 : nb
  body = bodies(i);
  qBody = q(body.position_num);
  vBody = v(body.velocity_num);
  if compute_gradients
    [SdotV{i}, dSdotVdq{i}] = motionSubspaceDotV(body, qBody, vBody);
  else
    SdotV{i} = motionSubspaceDotV(body, qBody, vBody);
  end
end
end

function [JdotV, dJdotVdq] = computeJacobianDotV(bodies, T, twists, SdotV, dTdq, dtwistsdq, dSdotVdq)
compute_gradients = nargout > 1;

world = 1;
nb = length(bodies);
if compute_gradients
  nq = size(dTdq{end}, 2);
end

JdotV = cell(1, nb);
JdotV{1} = zeros(6, 1);
if compute_gradients
  dJdotVdq = cell(1, nb);
  dJdotVdq{1} = zeros(numel(JdotV{1}, nq));
end

for i = 2 : nb
  body = bodies(i);
  parent = body.parent;
  parent_accel = JdotV{parent};
  
  if compute_gradients
    dparent_accel = dJdotVdq{parent};
    dSdotVdqi = zeros(numel(SdotV{i}), nq);
    dSdotVdqi(:, body.position_num) = dSdotVdq{i};
    [joint_accel, djoint_accel] = transformSpatialAcceleration(SdotV{i}, T, twists, parent, i, i, world, dSdotVdqi, dTdq, dtwistsdq);
    dJdotVdq{i} = dparent_accel + djoint_accel;
  else
    joint_accel = transformSpatialAcceleration(SdotV{i}, T, twists, parent, i, i, world);
  end
  JdotV{i} = parent_accel + joint_accel;
end
end