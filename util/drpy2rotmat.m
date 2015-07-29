function [ dR ] = drpy2rotmat(rpy)
% Returns a 9x3 which is the derivative with respect to rpy (roll, pitch,
% yaw) of the rotation matrix R

r = rpy(1);
p = rpy(2);
y = rpy(3);

dR = [0 cos(y)*-sin(p) cos(p)*-sin(y) ;
      0 -sin(p)*sin(y) cos(y)* cos(p) ;
      0 -cos(p) 0 ;
      cos(y)*sin(p)*cos(r)-sin(y)*-sin(r) cos(y)*cos(p)*sin(r) -sin(y)*sin(p)*sin(r)-cos(y)*cos(r) ;
      sin(y)*sin(p)*cos(r)+cos(y)*-sin(r) sin(y)*cos(p)*sin(r)  cos(y)*sin(p)*sin(r)-sin(y)*cos(r) ;
      cos(p)*cos(r) -sin(p)*sin(r) 0 ;
      cos(y)*sin(p)*-sin(r)+sin(y)*cos(r) cos(y)*cos(p)*cos(r) -sin(y)*sin(p)*cos(r)+cos(y)*sin(r) ;
      sin(y)*sin(p)*-sin(r)-cos(y)*cos(r) sin(2)*cos(p)*cos(r)  cos(2)*sin(p)*cos(r)+sin(y)*sin(r) ;
      cos(p)*-sin(r) -sin(p)*cos(r) 0];

end


