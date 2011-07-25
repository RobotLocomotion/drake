function c = runLQR(p)

if (nargin<1)
  p = PlanarQuadPlant;
end

x0 = zeros(6,1);
u0 = p.m*p.g/2 * [1;1];
Q = diag([10 10 10 1 1 (p.L/2/pi)]);  %Q = diag([10*ones(1,3) ones(1,3)]);
R = [0.1 0.05; 0.05 0.1];  %R = diag([0.1 0.1]);

[c,V0] = tilqr(p,x0,u0,Q,R);

sys = feedback(p,c);

pp = sys.taylorApprox(0,x0,[],3);  % make polynomial approximation
options=struct();
options.degL1=2;
%options.method='bilinear';
%options.degV=4;
V=regionOfAttraction(pp,x0,2*V0,options);

figure(1); plotFunnel(V,x0,[3 6]); 
xlabel('$\theta$','interpreter','latex');
ylabel('$\dot\theta$','interpreter','latex');

if (nargout<1)

  v = PlanarQuadVisualizer();
  
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
