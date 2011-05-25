function runROA

p = CartPolePlant;
p = p.setInputLimits(-inf,inf);
v = CartPoleVisualizer;

% up fp:
c = CartPoleLQR(p);
sys = feedback(p,c);
x0=[0;pi;0;0];

% down fp (no feedback):
%sys=p;
%x0 = zeros(2,1);

pp = sys.taylorApprox(0,x0,[],3);  % make polynomial approximation

figure(1); clf; hold on; axis([-pi/2,3*pi/2,-5,5]);

options.degL1=1;
%options.method = 'bilinear';
V=regionOfAttraction(pp,x0,c.S,options);

plotFunnel(V,x0);

options.method = 'pablo';
V=regionOfAttraction(pp,x0,c.S,options);

plotFunnel(V,x0);

%return;

xroa=getLevelSet(V,x0);
for i = 1:5
  x=xroa(:,ceil(size(xroa,2)*rand));  % pick random point on the boundary
  xinside=.99*x+.01*x0;  % make a point just inside
  xtraj=simulate(sys,[0 3],xinside);
%  playback(v,xtraj);
  breaks=xtraj.getBreaks();
  xf = xtraj.eval(breaks(end));
  if (any(abs(xf-x0)>.1)) 
    xinside
    xf
    xf-x0
    xt=xtraj.eval(xtraj.getBreaks());
    figure(1); hold on;
    plot(xt(1,:),xt(2,:),'g.-');
    keyboard;
    error('Verified initial condition didn''t make it to the goal'); 
  end
  
%  xoutside=1.1*x-.1*x0;  % make a point just outside
%  xtraj=simulate(sys,[0 3],xoutside);
%  playback(v,xtraj);
end

disp('simulation verification passed');

