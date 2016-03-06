function globalStability

% Use SOS to find a Lyapunov function to certify the global stability of
% the damped pendulum.  Uses trig to poly substitution to perform exact
% analysis.  Recovers a Lyapunov function that looks a lot like the energy,
% but is better!  (doesn't require LaSalle's theorem)

checkDependency('spotless');
checkDependency('mosek');

% set up state variables
s = msspoly('s',1);
c = msspoly('c',1);
thetadot = msspoly('w',1);
x = [s;c;thetadot];

% polynomial dynamics (using parameters from the plant class) in terms of
% s,c, and thetadot
p = PendulumPlant();
f = [ c*thetadot; -s*thetadot; (-p.b*thetadot - p.m*p.g*p.l*s)/(p.m*p.l.^2) ];
x0 = [0;1;0];
eps = 1e-4;

prog = spotsosprog;
prog = prog.withIndeterminate(x);

deg_V = 2;
[prog,V] = prog.newFreePoly(monomials(x,0:deg_V));
prog = prog.withSOS(V - eps*(x-x0)'*(x-x0));  % V is strictly positive away from x0
Vdot = diff(V,x)*f; 

deg_lambda = 2;
[prog,lambda] = prog.newFreePoly(monomials(x,0:deg_lambda));

% Vdot is strictly negative away from x0 (but make an exception for the
% upright fixed point by multipling by s^2)
prog = prog.withSOS( -Vdot - lambda*(s^2+c^2-1) - eps*(x-x0)'*(x-x0)*s^2 );  

prog = prog.withEqs( subs(V,x,x0) );  % V(0) = 0

solver = @spot_mosek;
%solver = @spot_sedumi;
options = spot_sdp_default_options();
options.verbose = 1;

sol = prog.minimize(0,solver,options);

V = sol.eval(V)
Vdot = sol.eval(Vdot);


[Theta,ThetaDot] = meshgrid(-pi:0.1:pi, -8:0.25:8);  % for surfs

% first plot contours

figure(1);
subplot(1,2,1);
ezcontour(@(theta,thetadot)dmsubs(V,x,[sin(theta');cos(theta');thetadot']),[-2*pi,2*pi,-8,8]);
title('$$ V $$','interpreter','latex','fontsize',20) 
xlabel('$$ \theta $$','interpreter','latex','fontsize',15)
ylabel('$$ \dot{\theta} $$','interpreter','latex','fontsize',15)
subplot(1,2,2);
Vmesh = dmsubs(V,x,[sin(Theta(:)');cos(Theta(:)');ThetaDot(:)']);
surf(Theta,ThetaDot,reshape(Vmesh,size(Theta)));
title('$$ V $$','interpreter','latex','fontsize',20) 
xlabel('$$ \theta $$','interpreter','latex','fontsize',15)
ylabel('$$ \dot{\theta} $$','interpreter','latex','fontsize',15)

figure(2);
subplot(1,2,1);
ezcontour(@(theta,thetadot)dmsubs(Vdot,x,[sin(theta');cos(theta');thetadot']),[-2*pi,2*pi,-8,8]);
title('$$ \dot{V} $$','interpreter','latex','fontsize',20) 
xlabel('$$ \theta $$','interpreter','latex','fontsize',15)
ylabel('$$ \dot{\theta} $$','interpreter','latex','fontsize',15)
subplot(1,2,2);
Vdotmesh = dmsubs(Vdot,x,[sin(Theta(:)');cos(Theta(:)');ThetaDot(:)']);
surf(Theta,ThetaDot,reshape(Vdotmesh,size(Theta)));
title('$$ \dot{V} $$','interpreter','latex','fontsize',20) 
xlabel('$$ \theta $$','interpreter','latex','fontsize',15)
ylabel('$$ \dot{\theta} $$','interpreter','latex','fontsize',15)

end
