function TJ = Tjcalc(pitch,q)
% single joint kinematic transformation
% this is the kinematic analog to jcalc from the spatial vector library

if pitch == 0				% revolute joint
  TJ = [rotz(q),zeros(3,1);zeros(1,3),1];
elseif pitch == inf			% prismatic joint
  TJ = [eye(3),[0;0;q];[0 0 0 1]];
else					% helical joint
  TJ = [rotz(q),[0;0;q*pitch];zeros(1,3),1];
end
