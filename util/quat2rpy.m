function rpy = quat2rpy(q)
% convert quaternion to ROS rpy 
% from http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
% note, that this is equivalent to the aerospace toolbox
% quat2angle(q','XYZ');

w=q(1);x=q(2);y=q(3);z=q(4);

rpy = [atan2(2*(w*x + y*z), 1-2*(x*x +y*y)); ...
  asin(2*(w*y - z*x)); ...
  atan2(2*(w*z + x*y), 1-2*(y*y+z*z))];

