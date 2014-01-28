function c = runLQR

p = PlanarQuadPlant;
v = PlanarQuadVisualizer(p);

if checkDependency('sedumi')
  [c,V] = hoverLQR(p);
  sys = feedback(p,c);
  
  figure(1); plotFunnel(V,struct('plotdims',[3 6]));
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
    
    ts = xtraj.getBreaks();Vs=zeros(1,length(ts));
    for i=1:length(ts)
      Vs(i) = V.eval(ts(i),xtraj.eval(ts(i)));
    end
    if any(diff(Vs)>1e-4)
      diff(Vs)
      error('V(t,x) increased at some point in the trajectory from an initial condition in the verified ROA.');
    end
  end
  
else
  c = hoverLQR(p);
  sys = feedback(p,c);
  
  for i=1:5
    xtraj = simulate(sys,[0 4]);
    v.playback(xtraj);
  end

end


