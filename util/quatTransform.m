function [quat,dquat] = quatTransform(u,v)
% returns the quaternion that rotates unit vector u to unit vector v
if(abs(norm(u)-1)>1e-10||abs(norm(v)-1)>1e-10)
  error('u and v must be unit vector');
end
quat_axis = u+v;
len_quat_axis = norm(quat_axis);
if(nargout>1)
  dquat_axis = [eye(3) eye(3)];
end
if(len_quat_axis == 0)
  rand_vec = randn(3,1);
  quat_axis = rand_vec-u*u'*rand_vec;
  len_quat_axis = norm(quat_axis);
  if(nargout>1)
    dquat_axis = -2*(u'*rand_vec)*[eye(3) zeros(3)];
  end
end
quat = [0;quat_axis/len_quat_axis];
if(nargout>1)
  dlen_quat_axis = quat_axis'/len_quat_axis*dquat_axis;
  dquat = [zeros(1,6);(dquat_axis*len_quat_axis-quat_axis*dlen_quat_axis)/(len_quat_axis^2)];
end
end