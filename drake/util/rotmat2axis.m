function a = rotmat2axis(R)

% convert rotation matrix to axis angle representation:
%  from http://en.wikipedia.org/wiki/Axis-angle_representation
%
% @param R is a 3x3 rotation matrix
% @retval a is a [x;y;z;theta]
%

theta = acos((trace(R)-1)/2);
if (theta>eps)
  a = [1/(2*sin(theta))*[ R(3,2) - R(2,3); ...
    R(1,3) - R(3,1); ...
    R(2,1) - R(1,2)]; ...
    theta];
else
  a = [1;0;0;0];
end
  
end