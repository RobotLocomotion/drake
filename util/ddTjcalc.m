function ddTJ = ddTjcalc(pitch,q)
% second time derivative of single joint kinematic transformation 

if pitch == 0				% revolute joint
  s=sin(q); c=cos(q);
  ddTJ = [-c,s,0,0; -s,-c,0,0; zeros(2,4)];
elseif pitch == inf			% prismatic joint
  ddTJ = zeros(4,4);
else					% helical joint
  s=sin(q); c=cos(q);
  ddTJ = [-c,s,0,0; -s,-c,0,0; zeros(2,4)];
end
