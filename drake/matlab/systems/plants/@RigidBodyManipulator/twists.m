function ret = twists(obj, transforms, q, v)
bodies = obj.body;
nb = length(bodies);
ret = cell(1, nb);

for i = 1 : nb
  body = bodies(i);
  if body.parent > 0 && ~isempty(v)
    parentTwist = ret{body.parent};
    q_body = q(body.position_num);
    v_Body = v(body.velocity_num);
    jointTwist = transformTwists(transforms{i}, motionSubspace(body, q_body) * v_Body);
    ret{i} = parentTwist + jointTwist;
  else
    twistSize = 6;
    ret{i} = zeros(twistSize, 1);
  end
end
end
