function runROA

p = PendulumPlant;
v = PendulumVisualizer;
x0=[pi;0];

%% for testing polytopic code
%pp = p.taylorApprox(0,x0,0,3);
%c = PendulumLQR(pp);
%pp = pp.setInputLimits(-inf,inf);  % just for testing
%sys=feedback(pp,c);
%figure(); clf; axis([c.x0(1)+[-pi,pi],c.x0(2)+[-5,5]]);
%plotRegions(sys); 
%%

p = p.setInputLimits(-inf,inf);  % for now
c = PendulumLQR(p);
sys = feedback(p,c);

pp = sys.taylorApprox(0,x0,[],3);  % make polynomial approximation

figure(1); clf; hold on; axis([-pi/2,3*pi/2,-5,5]);

options=struct();
%options.method='sampling';
%options.method='bilinear';
options.method='levelSet'
V=regionOfAttraction(pp,x0,[],options);
xroa=getLevelSet(V,x0);
plot(xroa(1,:),xroa(2,:),'r');

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

