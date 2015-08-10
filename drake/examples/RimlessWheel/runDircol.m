function runDircol
% A simple example of using direct collocation to
% find a limit cycle

N = 11;

rw = RimlessWheelPlant;
prog = DircolTrajectoryOptimization(rw.getMode(1),N,[.5 2]);

% theta(1) = gamma - alpha
prog = prog.addStateConstraint(ConstantConstraint(rw.gamma - rw.alpha),1,1);

% theta(N) = gamma + alpha
prog = prog.addStateConstraint(ConstantConstraint(rw.gamma + rw.alpha),N,1);

% thetadot(1) = cos(2*alpha)*thetadot(N)
prog = prog.addConstraint(LinearConstraint(0,0,[cos(2*rw.alpha),-1]),[prog.x_inds(2,N);prog.x_inds(2,1)]);

% for i 2,...,N-1   gamma-alpha < theta(i) < gamma+alpha
prog = prog.addStateConstraint(BoundingBoxConstraint(rw.gamma-rw.alpha,rw.gamma+rw.alpha),2:N-1,1);

% x position is zero
prog = prog.addStateConstraint(ConstantConstraint(0),1,3);

% display homoclinic orbic
theta_samples = rw.gamma + 2*rw.alpha*[-1:.01:1];
thetadot_homoclinic = orbit(rw.getMode(1),theta_samples,9.81);
thetadot_orbit2 = orbit(rw.getMode(1),theta_samples,9);
thetadot_orbit3 = orbit(rw.getMode(1),theta_samples,11);
% display vector field
[Theta,ThetaDot] = meshgrid(theta_samples(1:10:end),linspace(0,2,21));
xdot = rw.getMode(1).dynamics(0,[Theta(:)';ThetaDot(:)';0*Theta(:)']);
ThetaDDot = reshape(xdot(2,:),size(Theta));

  function draw(dt,x,u)
    quiver(Theta,ThetaDot,ThetaDot,ThetaDDot,'r');
    hold on;
    plot(x(1,:),x(2,:),'b.-','LineWidth',2);
    plot(theta_samples,thetadot_homoclinic,'g',...
      theta_samples,real(thetadot_orbit2),'g',...
      theta_samples,thetadot_orbit3,'g','LineWidth',3);
    line(rw.gamma-rw.alpha*[1,1],[0 2],'Color','k','LineWidth',2);
    line(rw.gamma+rw.alpha*[1,1],[0 2],'Color','k','LineWidth',2);
    line([x(1,1),x(1,N)],[x(2,1),x(2,N)],'Color','b','LineStyle','--','LineWidth',2);
%    axis equal;
    hold off;
    axis([rw.gamma - 2*rw.alpha,rw.gamma+2*rw.alpha,0 2]);
    drawnow;
  end
prog = prog.addTrajectoryDisplayFunction(@draw);

prog.solveTraj(1);

end
