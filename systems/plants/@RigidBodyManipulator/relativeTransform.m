function [T, dTdq] = relativeTransform(obj, kinsol, base, body)

compute_gradient = nargout > 1;

[base, Tbase_frame] = extractFrameInfo(obj, base);
[end_effector, Tbody_frame] = extractFrameInfo(obj, body);

Tbaseframe_to_world = kinsol.T{base} * Tbase_frame;
Tworld_to_baseframe = homogTransInv(Tbaseframe_to_world);
Tbodyframe_to_world = kinsol.T{end_effector} * Tbody_frame;
T = Tworld_to_baseframe * Tbodyframe_to_world;

if compute_gradient
  dTbaseframe_to_world = matGradMult(kinsol.dTdq{base}, Tbase_frame);
  dTworld_to_baseframe = dinvT(Tbaseframe_to_world, dTbaseframe_to_world);
  dTbodyframe_to_world = matGradMult(kinsol.dTdq{end_effector}, Tbody_frame);
  dTdq = matGradMultMat(Tworld_to_baseframe, Tbodyframe_to_world, dTworld_to_baseframe, dTbodyframe_to_world);
end

end