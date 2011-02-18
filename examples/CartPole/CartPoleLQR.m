function sys = CartPoleLQR(plant)

typecheck(plant,'DynamicalSystem');

Q = diag([100,10,1,1]); R = 10;
xG = [0;pi;0;0]; uG = 0;
sys = tilqr(plant,xG,uG,Q,R);

% NOTEST