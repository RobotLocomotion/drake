function c = runLQR

p = PlanarQuadPlant;
v = PlanarQuadVisualizer();

[c,V] = hoverLQR(p);

sys = feedback(p,c);
%for i=1:5
%  xtraj = simulate(sys,[0 4]);
%  v.playback(xtraj);
%end
%return;

figure(1); plotFunnel(V,[],[3 6]); 
xlabel('$\theta$','interpreter','latex');
ylabel('$\dot\theta$','interpreter','latex');


n=5;
x0=zeros(6,1);
y=getLevelSet(V,[],struct('num_samples',n));
for i=1:n
  xtraj=simulate(sys,[0 4],x0 + .99*y(:,i));
  figure(1); fnplt(xtraj,[3 6]);
  figure(25);
  v.playback(xtraj);
  if (norm(xtraj.eval(4)-x0)>1e-2)
    error('initial condition from verified ROA didn''t get to the top (in 4 seconds)');
  end
end
