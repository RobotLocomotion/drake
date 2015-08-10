function TJdot = Tjcalcpdot(jcode,q,qd)
% time derivative of single joint kinematic transformation for planar models
% this is the kinematic analog to jcalcp from the spatial vector library


switch (jcode)
  case 1 % pin joint
    c = qd*cos(q);
    s = qd*sin(q);
    TJdot = [-s, -c 0; c -s 0; 0 0 0];
  case 2 % x-axis prismatic
    TJdot = [0 0 qd; 0 0 0; 0 0 0];
  case 3 % y-axis prismatic
    TJdot = [0 0 0; 0 0 qd; 0 0 0];
end
