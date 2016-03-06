function trajectoryClosestPointTest

p = addpath(fullfile(pwd,'..'));

r = PendulumPlant;
[utraj,xtraj] = swingUpTrajectory(r);

figure(1); clf;
fnplt(xtraj); axis equal
hold on;

dxtraj = fnder(xtraj);

N = 20;
for i=1:N
  xs = diag([2*pi,14])*rand(2,1) - [pi;7];
  [xnear,tnear] = closestPoint(xtraj,Point(r.getStateFrame(),xs));
  plot([xs(1),xnear(1)],[xs(2),xnear(2)],'r.-');

  dxnear = dxtraj.eval(tnear);
  xnearn = xnear + .1*dxnear;
  line([xnear(1),xnearn(1)],[xnear(2),xnearn(2)],'Color','g');
  
  if any(abs(xtraj.tspan - tnear) < 1e-6)
    continue; % don't expect perp test below to pass at endpoints
  end
  
  if abs(dot(dxnear,xnear-xs)/(norm(dxnear)*norm(xnear-xs)))>1e-4
    error('should be perpendicular at closest point');
  end
end

path(p);

end