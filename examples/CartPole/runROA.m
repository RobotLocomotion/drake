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

pp = sys.taylorApprox(0,x0,0,3);  % make polynomial approximation

V=regionOfAttraction(pp);

xroa=getLevelSet(subs(V,pp.p_x([1 3]),zeros(2,1)));
figure(1);
plot(xroa(1,:)+x0(1),xroa(2,:)+x0(2),'r');
return;

for i = 1:5
  x=x0+xroa(:,ceil(size(xroa,2)*rand));  % pick random point on the boundary
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

