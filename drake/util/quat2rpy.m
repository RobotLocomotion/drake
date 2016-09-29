function rpy = quat2rpy(q)
% convert quaternion to ROS rpy 
% from http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

q=q/norm(q);
w=q(1);x=q(2);y=q(3);z=q(4);
% rpy = [atan2(2*(w*x + y*z), 1-2*(x*x +y*y)); ...
%   asin(2*(w*y - z*x)); ...
%   atan2(2*(w*z + x*y), 1-2*(y*y+z*z))];

% more stable than previous form (old one did not work for quat2rpy(rpy2quat([0,-pi/2,pi])))
rpy = [atan2(2*(w*x + y*z), w*w + z*z -(x*x +y*y)); ...
  asin(2*(w*y - z*x)); ...
  atan2(2*(w*z + x*y), w*w + x*x-(y*y+z*z))];

