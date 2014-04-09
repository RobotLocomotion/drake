%NOTEST
function q = getRandomConfiguration(manipulator)
q = zeros(manipulator.getNumPositions(), 1);
for i = 2 : manipulator.getNumBodies()
  body = manipulator.body(i);
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