function runHinf

d = CartPolePlant;
d = setInputLimits(d,-inf,inf);
v = CartPoleVisualizer(d);
[c,V] = balanceHinf(d);

sys = feedback(d,c); 

x0 = [0;pi;0;0];
%for i=1:5
%  xtraj=simulate(sys,[0 4],x0+.2*randn(4,1));
%  v.playback(xtraj);
%end

V = V.inFrame(d.getStateFrame);

figure(1); plotFunnel(V,struct('inclusion','projection','plotdims',[2 4])); 
xlabel('$\theta$','interpreter','latex');
ylabel('$\dot\theta$','interpreter','latex');


n=5;
y=getLevelSet(V,[],struct('num_samples',n));
for i=1:n
  xtraj=simulate(sys,[0 4],.01*x0 + .99*y(:,i));
  figure(1); fnplt(xtraj,[2 4]);
  figure(25);
  v.playback(xtraj);
  if (V.eval(4,xtraj.eval(4))>V.eval(0,xtraj.eval(0)))
    xtraj.eval(4)-x0
    error('simulation appears to be going uphill on the Lyapunov function');
  end
end

end
