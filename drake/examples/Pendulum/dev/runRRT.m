function pendulum_rrt

p = PendulumPlant();
v = PendulumVisualizer();
x0 = [0;0];
xG = [pi;0];

xlim = [-pi/2,3*pi/2; -10,10];  bWrap = [1;0];
ubins = {linspace(p.umin,p.umax,7)};
 
figure(1); clf;
euclidean_dist = inline('sum([min(x(1)+2*pi - Y(1,:),x(1) - Y(1,:));x(2)-Y(2,:)].^2,1);','x','Y');
options = struct('dt',.05,'converged',.25);
[xtraj,utraj] = oldrrt(p,euclidean_dist,x0,xG,xlim,bWrap,ubins,options);
playback(v,xtraj);
