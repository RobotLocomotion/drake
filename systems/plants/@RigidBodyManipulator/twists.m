function ret = twists(obj, transforms, q, v)
bodies = obj.body;
nb = length(bodies);
ret = cell(1, nb);

for i = 1 : nb
  body = bodies(i);
  if body.parent > 0 && ~isempty(v)
    parentTwist = ret{body.parent};
    qBody = q(body.position_num);
    vBody = v(body.velocity_num);
    jointTwist = motionSubspace(body, qBody) * vBody;
    ret{i} = transformTwists(transforms{i} \ transforms{body.parent}, parentTwist) + jointTwist;
  else
    twistSize = 6;
    ret{i} = zeros(twistSize, 1);
  end
end
end

% function Tdot = computeTdots(T, twist)
% Tdot = cell(length(T), 1);
% for i = 1 : length(T)
%   Tdot{i} = T{i} * twistToTildeForm(twist{i});
% end
% end