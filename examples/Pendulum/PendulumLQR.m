function [c,V] = PendulumLQR(plant)

x0=[pi;0];
u0=0;
Q = diag([10 1]);
R = 1;

[c,V] = tilqr(plant,x0,u0,Q,R);

if (nargout>1)
  plant = plant.setInputLimits(-inf,inf);  % for now
  pp = feedback(plant.taylorApprox(0,x0,u0,3),c);
  options=struct();
  options.method='levelSet'
  V=regionOfAttraction(pp,x0,[],options);
end

% NOTEST
