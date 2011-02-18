function phasePortrait(sys)

figure(1);
clf;

[Q,Qdot] = ndgrid(linspace(-3*pi/2,3*pi/2,21),linspace(-4,4,21));
Qddot = Q;
for i=1:prod(size(Q))
  xdot = sys.dynamics(0,[Q(i);Qdot(i)],0);
  Qddot(i) = xdot(2);
end

quiver(Q,Qdot,Qdot,Qddot);

% NOTEST