function [dTJdotdq, dTJdotdqdot] = dTjcalcpdot(jcode,q,qd)
% time derivative of single joint kinematic transformation for planar models
% this is the kinematic analog to jcalcp from the spatial vector library


switch (jcode)
  case 1 % pin joint
    c = cos(q);
    s = sin(q);
    dTJdotdq = [-c*qd, s*qd 0; -s*qd -c*qd 0; 0 0 0];
    dTJdotdqdot = [-s, -c 0; c -s 0; 0 0 0]; 
  case 2 % x-axis prismatic
    dTJdotdq = [0 0 0; 0 0 0; 0 0 0];
    dTJdotdqdot = [0 0 1; 0 0 0; 0 0 0];
  case 3 % y-axis prismatic
    dTJdotdq = [0 0 0; 0 0 0; 0 0 0];
    dTJdotdqdot = [0 0 0; 0 0 1; 0 0 0];
end
