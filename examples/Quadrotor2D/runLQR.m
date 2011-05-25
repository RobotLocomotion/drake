function c = runLQR(p)

if (nargin<1)
  p = PlanarQuadPlant;
end

x0 = zeros(6,1);
u0 = p.m*p.g/2 * [1;1];
Q = diag([10*ones(1,3) ones(1,3)]);
R = diag([0.1 0.1]);

c = tilqr(p,x0,u0,Q,R);

sys = feedback(p,c);

pp = sys.taylorApprox(0,x0,[],3);  % make polynomial approximation
options.degL1=2;
tic
V=regionOfAttraction(pp,x0,c.S,options);
toc
figure(1); plotFunnel(V,x0,[3 6]);

if (nargout<1)
  
for i=1:5  
  v = PlanarQuadVisualizer();
  
  xtraj{i}=simulate(sys,[0 4]);
  xtraj{i}=shiftTime(xtraj{i},4*(i-1));
end

xtraj = HybridTrajectory(xtraj);
v.playback(xtraj);
%v.playbackAVI(xtraj,'PlanarQuadLQR');

end
