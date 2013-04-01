function q=rpy2quat(rpy)
% converts ROS's rpy representation of angles to unit quaternions
% from https://code.ros.org/trac/ros-pkg/ticket/4247, but with
%  q = [w;x;y;z]

typecheck(rpy,'numeric');
sizecheck(rpy,3);

s=sin(rpy/2); c=cos(rpy/2);

q = [ c(1)*c(2)*c(3) + s(1)*s(2)*s(3); ...
      s(1)*c(2)*c(3) - c(1)*s(2)*s(3); ...
      c(1)*s(2)*c(3) + s(1)*c(2)*s(3); ...
      c(1)*c(2)*s(3) - s(1)*s(2)*c(3) ];

q = q./(norm(q)+eps);


