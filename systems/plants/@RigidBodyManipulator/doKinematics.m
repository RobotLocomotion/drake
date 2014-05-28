function kinsol = doKinematics(model, q, compute_gradients, use_mex, v, compute_JdotV)
% kinsol=doKinematics(model, q, compute_JdotV, use_mex, v)
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
    kinsol.twists = computeTwistsInBaseFrame(bodies, kinsol.T, S, v);
    % TODO: remove once there are no more references to this:
    kinsol.Tdot = computeTdots(kinsol.T, kinsol.twists);
    
    if compute_JdotV
      SdotV = computeMotionSubspacesDotV(bodies, q, v);
      kinsol.JdotV = computeJacobianDotV(bodies, kinsol.T, kinsol.twists, SdotV);
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
  dHToParentdq = zeros(numel(H{i}), nq);
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
  dSdqi = zeros(numel(Si), nq);
  dSdqi(:, body.position_num) = dSdq{i};
  ret{i} = dAdH_times_X(H{i}, Si, dHdq{i}, dSdqi);
end
end

function twists = computeTwistsInBaseFrame(bodies, transforms, S, v)
nb = length(bodies);
twists = cell(1, nb);

for i = 1 : nb
  body = bodies(i);
  if body.parent > 0
    parentTwist = twists{body.parent};
    vBody = v(body.velocity_num);
    jointTwist = S{i} * vBody;
    twists{i} = parentTwist + transformTwists(transforms{i}, jointTwist);
  else
    twistSize = 6;
    twists{i} = zeros(twistSize, 1);
  end
end
end

function Tdot = computeTdots(T, twist)
Tdot = cell(length(T), 1);
for i = 1 : length(T)
  Tdot{i} = twistToTildeForm(twist{i}) * T{i};
end
end

function SdotV = computeMotionSubspacesDotV(bodies, q, v)
nb = length(bodies);
SdotV = cell(1, nb);
for i = 2 : nb
  body = bodies(i);
  qBody = q(body.position_num);
  vBody = v(body.velocity_num);
  SdotV{i} = motionSubspaceDotV(body, qBody, vBody);
end
end

function JdotV = computeJacobianDotV(bodies, transforms, twists, SdotV)
world = 1;
nb = length(bodies);
JdotV = cell(1, nb);
JdotV{1} = zeros(6, 1);
for i = 2 : nb
  body = bodies(i);
  parent = body.parent;
  predecessorAccel = JdotV{parent};
  jointAccel = transformSpatialAcceleration(SdotV{i}, transforms, twists, parent, i, i, world);
  JdotV{i} = predecessorAccel + jointAccel;
end
end
