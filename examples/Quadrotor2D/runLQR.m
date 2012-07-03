function c = runLQR

p = PlanarQuadPlant;
v = PlanarQuadVisualizer();

[c,V] = hoverLQR(p);

%sys = feedback(p,c);
%for i=1:5
%  xtraj = simulate(sys,[0 4]);
%  v.playback(xtraj);
%end
%return;

figure(1); plotFunnel(V,[],[3 6]); 
xlabel('$\theta$','interpreter','latex');
ylabel('$\dot\theta$','interpreter','latex');


n=10;
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


% compute points on the boundary
x = pp.p_x;
S = double(diff(diff(V,x)',x))/2;

K = 5;
X = randn(2,K);
X = X./repmat(sqrt(sum(X.^2,1)),2,1);
X = chol(S([3 6],[3 6]))\X;
  X = [0 0 ; 0 0 ; 1 0; 0 0 ; 0 0; 0 1]*X;

  for i = 1:K
%    xtraj = simulate(sys,[0 1],X(:,i));
    xtraj=simulate(sys,[0 4]);
    figure(1); fnplt(xtraj,[3 6]);
    figure(25);
    v.playback(xtraj);
  end

end
