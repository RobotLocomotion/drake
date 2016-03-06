function phasePortrait(sys,hFig,linespec)

if (nargin<2) hFig=1; end
if (nargin<3) linespec=[]; end
figure(hFig);

u=0;
if (getNumInputs(sys)<1)  % closed loop systems have no inputs
  u=[];
end


[Q,Qdot] = ndgrid(linspace(-3*pi/2,3*pi/2,21),linspace(-4,4,21));
Qddot = Q;
for i=1:prod(size(Q))
  xdot = sys.dynamics(0,[Q(i);Qdot(i)],u);
  Qddot(i) = xdot(2);
end

quiver(Q,Qdot,Qdot,Qddot);

% NOTEST