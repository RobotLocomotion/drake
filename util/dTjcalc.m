function dTJ = dTjcalc(pitch,q)
% time derivative of single joint kinematic transformation for planar models
% this is the kinematic analog to jcalcp from the spatial vector library

if pitch == 0				% revolute joint
  s=sin(q); c=cos(q);
  dTJ = [-s,-c,0,0; c,-s,0,0; zeros(2,4)];
elseif pitch == inf			% prismatic joint
  dTJ = [zeros(4,3),[0;0;1;0]];
else					% helical joint
  s=sin(q); c=cos(q);
  dTJ = [-s,-c,0,0; c,-s,0,0; 0,0,0,pitch; zeros(1,4)];
end
