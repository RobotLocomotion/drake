function q = getRandomConfiguration(obj)
q = zeros(obj.getNumPositions(), 1);
for i = 2 : obj.getNumBodies()
  body = obj.body(i);
  if body.floating == 0
    if isinf(body.joint_limit_min) || isinf(body.joint_limit_max)
      qBody = randn;
    else
      qBody = body.joint_limit_min + rand * (body.joint_limit_max - body.joint_limit_min);
    end
  else
    pos = randn(3, 1);
    axis_angle = [randn(3, 1); (rand - 0.5) * 2 * pi];
    if body.floating == 1
      qBody = [pos; axis2rpy(axis_angle)];
    elseif body.floating == 2
      qBody = [pos; axis2quat(axis_angle)];
    else
      error('floating joint type not recognized');
    end
  end
  q(body.position_num) = qBody;
end
end