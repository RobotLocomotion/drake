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
  TJ = Tjcalc(body.pitch,qi);
end
end