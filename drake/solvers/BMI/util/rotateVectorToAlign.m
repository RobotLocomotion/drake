function R = rotateVectorToAlign(a,b)
% return the one possible rotation matrix R that rotates a to b
a1 = a/norm(a);
b1 = b/norm(b);
axis = cross(a1,b1);
if(norm(axis)>1e-4)
  axis = axis/norm(axis);
  angle = acos(a1'*b1);
  q = [cos(angle/2);axis*sin(angle/2)];
elseif(a1'*b1>0)
  q = [1;0;0;0];
elseif(a1'*b1<0)
  r = randn(3,1);
  axis = r-r'*a1*a1;
  axis = axis/norm(axis);
  q = [0;axis];
end
R = quat2rotmat(q);
end