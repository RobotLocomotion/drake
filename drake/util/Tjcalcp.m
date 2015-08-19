function TJ = Tjcalcp(jcode,q)
% single joint kinematic transformation for planar models
% this is the kinematic analog to jcalcp from the spatial vector library


switch (jcode)
  case 1 % pin joint
    TJ = [rotmat(q),zeros(2,1);0,0,1];
  case 2 % x-axis prismatic
    TJ = [1,0,q; 0,1,0; 0,0,1];
  case 3 % y-axis prismatic
    TJ = [1,0,0; 0,1,q; 0,0,1];
end
