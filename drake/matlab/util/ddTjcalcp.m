function ddTJ = ddTjcalcp(jcode,q)
% second derivative of single joint kinematic transformation for planar models
% this is the kinematic analog to jcalcp from the spatial vector library

switch (jcode)
  case 1 % pin joint
    c = cos(q);
    s = sin(q);
    ddTJ = [-c, s 0; -s -c 0; 0 0 0];
  otherwise 
    ddTJ = zeros(3);
end