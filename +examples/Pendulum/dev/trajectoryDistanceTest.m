function trajectoryDistanceTest

p = addpath(fullfile(pwd,'..'));

r = PendulumPlant;
[utraj,xtraj] = swingUpTrajectory(r);

% test gradients
options.grad_method = {'user','numerical'};
for i=1:10
  xs = randn(2,1);
  [z,dz] = geval(@xtraj.distanceSq,xs,options);
end

% plot distance
[X,Y] = meshgrid(linspace(-5,5,35),linspace(-6,9,35));

Z=X;
for i=1:numel(X)
  Z(i) = distance(xtraj,[X(i);Y(i)]);
end

figure(1); clf;
surf(X,Y,Z); hold on;
h=fnplot(xtraj);
set(h,'Color',[1 1 1]);



path(p);

end


