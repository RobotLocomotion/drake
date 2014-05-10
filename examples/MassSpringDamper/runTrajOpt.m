function [xtraj,utraj,exitflag] = runTrajOpt(x0,xf,tf,dt)

if (nargin<4)
    dt = .01;
end

p = RigidBodyManipulator('MassSpringDamper.urdf');

display('Simulating an initial guess')
xtraj0 = p.simulate([0 5],x0);
display('Done simulating')

x_nom = xtraj0.eval(0:dt:tf);
nx = size(x_nom,1);
nt = size(x_nom,2);
u_nom = zeros(1,nt-1);
nu = size(u_nom,1);
h_nom = dt*ones(1,nt-1);
x00 = [reshape(x_nom,1,[]),reshape(u_nom,1,[]),reshape(h_nom,1,[])]';

% creates a nonlinear program with full trajectory and full input as unknowns
nlp = NonlinearProgramWConstraintObjects(nt*nx + (nt-1)*nu + (nt-1));

for i = 1:(nt-1)
    % makes a constraint tying each pairs of successive states and the input joining them
    xind = [(i-1)*nx+(1:2*nx),nt*nx+(i-1)*nu+(1:nu),nt*nx+(nt-1)*nu+i];
    dyn_con = BackwardEulerRBMDynamicsConstraint(p);
    dyn_con = dyn_con.setName(repmat({sprintf('Dynamics constraint at time %d', i)},dyn_con.num_cnstr,1));
    nlp = nlp.addNonlinearConstraint(dyn_con,xind);
end

% making sure the timestep size is between .5*dt and 2*dt
hc = BoundingBoxConstraint(0.5*dt*ones(nt-1,1),2*dt*ones(nt-1,1));
nlp = nlp.addBoundingBoxConstraint(hc,nt*nx+(nt-1)*nu+1:nt*nx+(nt-1)*nu+(nt-1));

cost = NonlinearConstraint(0,Inf,(nt-1)*nu,@(u)cost_eval(u));
nlp = nlp.addCost(cost,nt*nx+(1:(nt-1)*nu));

epsilon0 = zeros(2,1);
x0_constraint = BoundingBoxConstraint(x0-epsilon0,x0+epsilon0);
nlp = nlp.addBoundingBoxConstraint(x0_constraint,1:nx);

epsilonf = zeros(2,1);
xf_constraint = BoundingBoxConstraint(xf-epsilonf,xf+epsilonf);
nlp = nlp.addBoundingBoxConstraint(xf_constraint,(nt-1)*nx+(1:nx));

display('Starting trajectory optimization')
tic
nlp = nlp.setSolver('snopt');

% nlp = nlp.setCheckGrad(true);
% x_test = randn(nlp.num_vars,1);
% [fgh,dfgh] = nlp.objectiveAndNonlinearConstraints(x_test);
% options.grad_method = 'numerical';
% [~,dfghnum] = geval(@objectiveAndNonlinearConstraints,nlp,x_test,options);
% err = dfgh-dfghnum;
% magerr = max(max(abs(err)));
% fprintf('Maximum error for random test: %d\n',magerr)

[sol,~,exitflag] = nlp.solve(x00);
toc

usol = reshape(sol(nt*nx+1:nt*nx+(nt-1)*nu),nu,[]);
xsol = reshape(sol(1:nt*nx),nx,[]);
hsol = sol(nt*nx+(nt-1)*nu+1:nt*nx+(nt-1)*nu+nt-1);
utraj = PPTrajectory(foh(cumsum(hsol),usol));
xtraj = PPTrajectory(foh([0;cumsum(hsol)],xsol));

xtraj = xtraj.setOutputFrame(p.getOutputFrame());
v = p.constructVisualizer();
v.playback(xtraj, struct('slider',true));

end

function [c,dc] = cost_eval(u)
c = u(:)'*u(:);
dc = 2*u(:)';
end