function [T, dTdq] = relativeTransform(obj, kinsol, base, body)
% Computes the transform that maps vectors from body to base
%
% @param kinsol solution structure obtained from doKinematics
% @param base an integer ID for a RigidBody or RigidBodyFrame
% (obtained via e.g., findLinkId or findFrameInd) signifying the base
% frame.
% @param body an integer ID for a RigidBody or RigidBodyFrame
% (obtained via e.g., findLinkId or findFrameInd) signifying the body
% frame.
%
% @retval T transform from body frame to base frame
% @retval dTdq gradient of T with respect to robot position vector q.

compute_gradient = nargout > 1;

[base, Tbase_frame] = extractFrameInfo(obj, base);
[end_effector, Tbody_frame] = extractFrameInfo(obj, body);

Tbaseframe_to_world = kinsol.T{base} * Tbase_frame;
Tworld_to_baseframe = homogTransInv(Tbaseframe_to_world);
Tbodyframe_to_world = kinsol.T{end_effector} * Tbody_frame;
T = Tworld_to_baseframe * Tbodyframe_to_world;

if compute_gradient
  dTbaseframe_to_world = matGradMult(kinsol.dTdq{base}, Tbase_frame);
  dTworld_to_baseframe = dHomogTransInv(Tbaseframe_to_world, dTbaseframe_to_world);
  dTbodyframe_to_world = matGradMult(kinsol.dTdq{end_effector}, Tbody_frame);
  dTdq = matGradMultMat(Tworld_to_baseframe, Tbodyframe_to_world, dTworld_to_baseframe, dTbodyframe_to_world);
end

end