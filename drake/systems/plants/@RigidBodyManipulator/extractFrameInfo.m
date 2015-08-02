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