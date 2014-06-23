function [T, dTdq] = relativeTransform(obj, kinsol, base, body)

compute_gradient = nargout > 1;

[base, Tbase_frame] = extractFrameInfo(obj, base);
[end_effector, Tbody_frame] = extractFrameInfo(obj, body);

Tbaseframe_to_world = kinsol.T{base} * Tbase_frame;
Tworld_to_baseframe = homogTransInv(Tbaseframe_to_world);
Tbodyframe_to_world = kinsol.T{end_effector} * Tbody_frame;
T = Tworld_to_baseframe * Tbodyframe_to_world;

if compute_gradient
  nq = obj.getNumPositions();
  dTbase_frame = zeros(numel(Tbase_frame), nq); % TODO: can be made more efficient due to zero gradient
  dTbaseframe_to_world = matGradMultMat(kinsol.T{base}, Tbase_frame, kinsol.dTdq{base}, dTbase_frame);
  dTworld_to_baseframe = dinvT(Tbaseframe_to_world, dTbaseframe_to_world);
  dTbody_frame = zeros(numel(Tbody_frame), nq); % TODO: can be made more efficient due to zero gradient
  dTbodyframe_to_world = matGradMultMat(kinsol.T{end_effector}, Tbody_frame, kinsol.dTdq{end_effector}, dTbody_frame);
  dTdq = matGradMultMat(Tworld_to_baseframe, Tbodyframe_to_world, dTworld_to_baseframe, dTbodyframe_to_world);
end

end

function [body_index, Tframe] = extractFrameInfo(obj, body_or_frame_index)
if (body_or_frame_index < 0)
  frame = obj.frame(-body_or_frame_index);
  body_index = frame.body_ind;
  Tframe = frame.T;
else
  body_index = body_or_frame_index;
  Tframe=eye(4);
end
end