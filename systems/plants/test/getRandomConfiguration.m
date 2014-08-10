function q = getRandomConfiguration(obj)
%NOTEST
% Generates a random configuration vector
%
% @ retval q random configuration vector that satisfies the state
% constraints of the robot. For one-dof joints with joint limits, a
% uniformly random position between the joint limits is generated. For
% one-dof joints without joint limits, a standard normal distribution is
% used. For floating joints, a uniform distribution on SO(3) is used and
% subsequently converted to the appropriate representation.
% (implementation: generate random axis of rotation by normalizing a
% normally distributed vector, then rotate about that vector by a random
% angle between -pi and pi)

q = zeros(obj.getNumPositions(), 1);
for i = 2 : obj.getNumBodies()
  body = obj.body(i);
  if body.floating == 0
    if isinf(body.joint_limit_min) && isinf(body.joint_limit_max)
      q_body = randn;
    elseif ~isinf(body.joint_limit_min) && ~isinf(body.joint_limit_max)
      q_body = body.joint_limit_min + rand * (body.joint_limit_max - body.joint_limit_min);
    else
      error('case of one-sided joint limits currently not handled')
    end
  else
    pos = randn(3, 1);
    axis_angle = [randn(3, 1); (rand - 0.5) * 2 * pi];
    if body.floating == 1
      q_body = [pos; axis2rpy(axis_angle)];
    elseif body.floating == 2
      q_body = [pos; axis2quat(axis_angle)];
    else
      error('floating joint type not recognized');
    end
  end
  q(body.dofnum) = q_body;
  % +++FLOATINGBASE FIXME use this instead:
%   q(body.position_num) = qBody;
end
end