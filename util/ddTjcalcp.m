function ddTJ = dTjcalcp(jcode,q)
% second derivative of single joint kinematic transformation for planar models
% this is the kinematic analog to jcalcp from the spatial vector library

switch (jcode)
  case 1 % pin joint
    c = cos(q);
    s = sin(q);
    ddTJ = [-c, s 0; -s -c 0; 0 0 0];
  case 2 % x-axis prismatic
  case 3 % y-axis prismatic
    ddTJ = zeros(3);
end