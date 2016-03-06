function TJ = jointTransform(body, qi)
% Computes the transform that transforms vectors in the frame just after
% the joint to the frame just before the joint.
% @param body a RigidBody object which is the successor of the joint
% @param qi the joint configuration vector for the joint
% @retval TJ the joint transform

if body.floating==1
  rx = rotx(qi(4)); ry = roty(qi(5)); rz = rotz(qi(6));
  TJ = [rz*ry*rx,qi(1:3);zeros(1,3),1];
elseif body.floating==2
  TJ = [quat2rotmat(qi(4:7)),qi(1:3);zeros(1,3),1];
else
  pitch = body.pitch;
  joint_axis = body.joint_axis;
  if pitch == 0				% revolute joint
    TJ = [axis2rotmat([joint_axis; qi]),zeros(3,1);zeros(1,3),1];
  elseif pitch == inf			% prismatic joint
    TJ = [eye(3),qi * joint_axis;[0 0 0 1]];
  else					% helical joint
    TJ = [axis2rotmat([joint_axis; qi]), qi * pitch * joint_axis;zeros(1,3),1];
  end
end
end