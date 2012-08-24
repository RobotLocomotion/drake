function TJ = Tjcalc(pitch,q)
% single joint kinematic transformation for planar models
% this is the kinematic analog to jcalcp from the spatial vector library

if pitch == 0				% revolute joint
  TJ = [rotz(q),zeros(3,1);zeros(1,3),1];
elseif pitch == inf			% prismatic joint
  TJ = [zeros(4,3),[0;0;q;1]];
else					% helical joint
  TJ = [rotz(q),[0;0;q*pitch];zeros(1,3),1];
end
