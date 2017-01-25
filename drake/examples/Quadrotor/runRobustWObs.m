function [p, v, xtraj4, utraj4] = runRobustWObs
%function [p, v, xtraj1, utraj1, xtraj2, utraj2, xtraj3, utraj3, xtraj4, utraj4] = runRobustWObs

% simple planning demo which takes the quadrotor from hover at y=0m to a new hover at
% y=10m with minimal thrust. Creates a forest set up to have the Quadrotor
% swerve between trees blocking its path to get to y=10m.

p = Quadrotor();

% The important trees to create swerving path
% p = addTree(p, [.5,.5,1.5], [0;0], pi/4);
% trees = [0;0];
p = addTree(p, [.8,.45,1.25], [.5;2.3], pi/4);
p = addTree(p, [.75,.9,1.75], [2.2;4.5], -pi/5);
p = addTree(p, [.85,.95,1.65], [-1.85;5.2], -pi/3.7);
p = addTree(p, [.5,.35,1.65], [0;5.8], -pi/6);
p = addTree(p, [.55,.65,1.5], [.25;7.5], 0);
p = addTree(p, [.55,.85,1.6], [-1.35;8.5], pi/3.7);

%tree_pos = [[.5;2.3], [2.2;4.5], [-1.85;5.2], [0;5.8], [.25;7.5], [-1.35;8.5]];
tree_pos = [[.495;2.3], [2.2;4.5], [-1.85;5.2], [0;5.8], [.25;7.5], [-1.35;8.5]];
tree_width = ((.1+.4*[.45 .9 .95 .5 .65 .85]')+.55).^2;
%tree_width = ((.1+.4*[.45 .9 .95 .5 .65 .85]')+.8).^2;

% Random trees to make forest bigger and more dense
%p = addTrees(p, 25);

% initial conditions
x0 = Point(getStateFrame(p));
x0.base_y = -1.5;
x0.base_z = .5;
x0.base_x = .5;
x0.base_ydot = 12.5/3;
u0 = double(nominalThrust(p));

% final conditions: translated in x
xf = x0;
xf.base_y = 11;
xg = double(xf);

v = constructVisualizer(p); %, struct('use_collision_geometry',true));
v.draw(0,double(x0));

N = 61;
tf0 = 3; % duration
duration = [tf0 tf0];
traj_init.x = PPTrajectory(foh([0,tf0],[double(x0),double(xf)]));
traj_init.u = ConstantTrajectory(u0);

%Disturbance + LQR Weights
D = diag([.2 .2 .05].^2);
E0 = .01^2*eye(12);
Q = blkdiag(10*eye(6), 1*eye(6));
R = .1*eye(4);
Qf = Q;

function [g,dg] = cost(h,x,u)
    Qc = .3*eye(12);
    Rc = 1*eye(4);
    c = .5*(u'*Rc*u + (x-xg)'*Qc*(x-xg));
    g = h*c;
    dg = [c, h*(x-xg)'*Qc, h*u'*Rc];
end

function [c, dc] = treeDistance(q)
    c = zeros(size(tree_pos,2),1);
    dc = zeros(size(tree_pos,2),2);
    for k = 1:length(c)
        c(k) = (q(1:2)-tree_pos(1:2,k))'*(q(1:2)-tree_pos(1:2,k));
        dc(k,:) = 2*(q(1:2)-tree_pos(1:2,k))';
    end
end

%Solve without trees first to use as initial guess
options.integration_method = DirtranTrajectoryOptimization.MIDPOINT;
prog0 = DirtranTrajectoryOptimization(p,N,duration,options);
prog0 = prog0.addStateConstraint(ConstantConstraint(double(x0)),1);
prog0 = prog0.addInputConstraint(ConstantConstraint(u0),1);
prog0 = prog0.addStateConstraint(ConstantConstraint(double(xf)),N);
prog0 = prog0.addInputConstraint(ConstantConstraint(u0),N);
%prog0 = prog0.addStateConstraint(BoundingBoxConstraint([0], [inf]), 2:(N-1), 3);
prog0 = prog0.addRunningCost(@cost);
prog0 = prog0.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
prog0 = prog0.setSolverOptions('snopt','minoroptimalitytolerance',1e-4);
prog0 = prog0.setSolverOptions('snopt','majorfeasibilitytolerance',1e-3);
prog0 = prog0.setSolverOptions('snopt','minorfeasibilitytolerance',1e-4);
prog0 = prog0.setSolverOptions('snopt','iterationslimit',100000);
%prog1 = prog1.setSolverOptions('snopt','majoritterationslimit',10000);
%prog1 = prog1.setSolverOptions('snopt','minoritterationslimit',5000);

prog0 = addPlanVisualizer(p,prog0);
tic
[xtraj0,utraj0] = prog0.solveTraj(tf0,traj_init);
toc
traj_init.x = xtraj0;
traj_init.u = utraj0;

tsamp0 = linspace(xtraj0.tspan(1), xtraj0.tspan(2), N);
xsamp0 = xtraj0.eval(tsamp0);
usamp0 = utraj0.eval(tsamp0);

% figure();
% subplot(3,1,1);
% plot(tsamp0, xsamp0(1,:));
% subplot(3,1,2);
% plot(tsamp0, xsamp0(2,:));
% subplot(3,1,3);
% plot(tsamp0, xsamp0(3,:));

figure();
subplot(4,1,1);
plot(tsamp0, usamp0(1,:));
subplot(4,1,2);
plot(tsamp0, usamp0(2,:));
subplot(4,1,3);
plot(tsamp0, usamp0(3,:));
subplot(4,1,4);
plot(tsamp0, usamp0(4,:));

N1 = 121;
options.integration_method = DirtranTrajectoryOptimization.MIDPOINT;
prog1 = RobustDirtranTrajectoryOptimization(p,N1,D,E0,Q,R,Qf,duration,options);
prog1 = prog1.addStateConstraint(ConstantConstraint(double(x0)),1);
prog1 = prog1.addInputConstraint(ConstantConstraint(u0),1);
prog1 = prog1.addStateConstraint(ConstantConstraint(double(xf)),N1);
prog1 = prog1.addInputConstraint(ConstantConstraint(u0),N1-1);
%prog1 = prog1.addStateConstraint(BoundingBoxConstraint([0], [inf]), 2:(N-1), 3);
prog1 = prog1.addRunningCost(@cost);
prog1 = prog1.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
prog1 = prog1.setSolverOptions('snopt','minoroptimalitytolerance',1e-4);
prog1 = prog1.setSolverOptions('snopt','majorfeasibilitytolerance',1e-3);
prog1 = prog1.setSolverOptions('snopt','minorfeasibilitytolerance',1e-4);
prog1 = prog1.setSolverOptions('snopt','superbasicslimit',5000);
prog1 = prog1.setSolverOptions('snopt','iterationslimit',100000);

collision_constraint = FunctionHandleConstraint(tree_width,inf*ones(6,1),2,@treeDistance);
collision_constraint.grad_method = 'user';
collision_constraint.grad_level = 1;
prog1 = prog1.addStateConstraint(collision_constraint,2:N1-1,1:2);

prog1 = addPlanVisualizer(p,prog1);
tic
[xtraj1,utraj1,z1,F1,info1] = prog1.solveTraj(tf0,traj_init);
toc

tsamp1 = linspace(xtraj1.tspan(1), xtraj1.tspan(2), N1);
xsamp1 = xtraj1.eval(tsamp1);
usamp1 = utraj1.eval(tsamp1);

options.integration_method = DirtranTrajectoryOptimization.MIDPOINT;
prog4 = RobustDirtranTrajectoryOptimization(p,N1,D,E0,Q,R,Qf,duration,options);
prog4 = prog4.addStateConstraint(ConstantConstraint(double(x0)),1);
prog4 = prog4.addInputConstraint(ConstantConstraint(u0),1);
prog4 = prog4.addStateConstraint(ConstantConstraint(double(xf)),N1);
prog4 = prog4.addInputConstraint(ConstantConstraint(u0),N1-1);
%prog4 = prog4.addStateConstraint(BoundingBoxConstraint([0], [inf]), 2:(N-1), 3);
prog4 = prog4.addRunningCost(@cost);
prog4 = prog4.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
prog4 = prog4.setSolverOptions('snopt','minoroptimalitytolerance',1e-4);
prog4 = prog4.setSolverOptions('snopt','majorfeasibilitytolerance',1e-3);
prog4 = prog4.setSolverOptions('snopt','minorfeasibilitytolerance',1e-4);
prog4 = prog4.setSolverOptions('snopt','superbasicslimit',5000);
prog4 = prog4.setSolverOptions('snopt','iterationslimit',100000);

tree_width2 = ((.1+.4*[.45 .9 .95 .5 .65 .85]')+.55 + .04689).^2;
collision_constraint2 = FunctionHandleConstraint(tree_width2,inf*ones(6,1),2,@treeDistance);
collision_constraint2.grad_method = 'user';
collision_constraint2.grad_level = 1;
prog4 = prog4.addStateConstraint(collision_constraint2,2:N1-1,1:2);

prog4 = addPlanVisualizer(p,prog4);
tic
[xtraj4,utraj4,z4,F4,info4] = prog4.solveTraj(tf0,traj_init);
toc

% tsamp4 = linspace(xtraj1.tspan(1), xtraj1.tspan(2), N1);
% xsamp4 = xtraj1.eval(tsamp4);
% usamp4 = utraj1.eval(tsamp4);
% 
% % figure();
% % subplot(3,1,1);
% % plot(tsamp1, xsamp1(1,:));
% % subplot(3,1,2);
% % plot(tsamp1, xsamp1(2,:));
% % subplot(3,1,3);
% % plot(tsamp1, xsamp1(3,:));
% % 
% figure();
% subplot(4,1,1);
% plot(tsamp1, usamp1(1,:));
% subplot(4,1,2);
% plot(tsamp1, usamp1(2,:));
% subplot(4,1,3);
% plot(tsamp1, usamp1(3,:));
% subplot(4,1,4);
% plot(tsamp1, usamp1(4,:));
% 
% v.playback(xtraj1, struct('slider',true));
% 
% D = diag([.2 .2 .05].^2);
% E0 = .01^2*eye(12);
% Q = blkdiag(10*eye(6), 1*eye(6));
% R = .1*eye(4);
% Qf = Q;
% N2 = 61;
% options.integration_method = DirtranTrajectoryOptimization.MIDPOINT;
% prog2 = RobustDirtranTrajectoryOptimization(p,N2,D,E0,Q,R,Qf,duration,options);
% prog2 = prog2.addRobustCost(Q,5*R,Qf);
% %prog2 = prog2.addRobustInputConstraint();
% prog2 = prog2.addRobustStateConstraint(collision_constraint,2:(N2-1),1:2);
% prog2 = prog2.addStateConstraint(ConstantConstraint(double(x0)),1);
% prog2 = prog2.addInputConstraint(ConstantConstraint(u0),1);
% prog2 = prog2.addStateConstraint(ConstantConstraint(double(xf)),N2);
% prog2 = prog2.addInputConstraint(ConstantConstraint(u0),N2-1);
% %prog2 = prog2.addStateConstraint(BoundingBoxConstraint([0], [inf]), 2:(N-1), 3);
% prog2 = prog2.addStateConstraint(collision_constraint,2:N2-1,1:2);
% prog2 = prog2.addRunningCost(@cost);
% prog2 = prog2.setSolverOptions('snopt','majoroptimalitytolerance',1e-2);
% prog2 = prog2.setSolverOptions('snopt','minoroptimalitytolerance',1e-4);
% prog2 = prog2.setSolverOptions('snopt','majorfeasibilitytolerance',1e-3);
% prog2 = prog2.setSolverOptions('snopt','minorfeasibilitytolerance',1e-4);
% prog2 = prog2.setSolverOptions('snopt','superbasicslimit',5000);
% prog2 = prog2.setSolverOptions('snopt','iterationslimit',100000);
% 
% prog2 = addPlanVisualizer(p,prog2);
% tic
% [xtraj2,utraj2,z2,F2,info2] = prog2.solveTraj(tf0,traj_init);
% toc
% 
% traj_init.x = xtraj2;
% traj_init.u = utraj2;
% 
% N2 = 121;
% options.integration_method = DirtranTrajectoryOptimization.MIDPOINT;
% prog2 = RobustDirtranTrajectoryOptimization(p,N2,D,E0,Q,R,Qf,duration,options);
% prog2 = prog2.addRobustCost(Q,5*R,Qf);
% %prog2 = prog2.addRobustInputConstraint();
% prog2 = prog2.addRobustStateConstraint(collision_constraint,2:(N2-1),1:2);
% prog2 = prog2.addStateConstraint(ConstantConstraint(double(x0)),1);
% prog2 = prog2.addInputConstraint(ConstantConstraint(u0),1);
% prog2 = prog2.addStateConstraint(ConstantConstraint(double(xf)),N2);
% prog2 = prog2.addInputConstraint(ConstantConstraint(u0),N2-1);
% prog2 = prog2.addStateConstraint(BoundingBoxConstraint([0], [inf]), 2:(N-1), 3);
% prog2 = prog2.addStateConstraint(collision_constraint,2:N2-1,1:2);
% prog2 = prog2.addRunningCost(@cost);
% prog2 = prog2.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
% prog2 = prog2.setSolverOptions('snopt','minoroptimalitytolerance',1e-4);
% prog2 = prog2.setSolverOptions('snopt','majorfeasibilitytolerance',1e-3);
% prog2 = prog2.setSolverOptions('snopt','minorfeasibilitytolerance',1e-4);
% prog2 = prog2.setSolverOptions('snopt','superbasicslimit',5000);
% prog2 = prog2.setSolverOptions('snopt','iterationslimit',100000);
% 
% prog2 = addPlanVisualizer(p,prog2);
% tic
% [xtraj2,utraj2,z2,F2,info2] = prog2.solveTraj(tf0,traj_init);
% toc
% 
% v.playback(xtraj2, struct('slider',true));
% 
% tsamp2 = linspace(xtraj2.tspan(1), xtraj2.tspan(2), N);
% xsamp2 = xtraj2.eval(tsamp2);
% usamp2 = utraj2.eval(tsamp2);
% 
% D = diag([.4 .4 .05].^2);
% E0 = .01^2*eye(12);
% Q = blkdiag(10*eye(6), 1*eye(6));
% R = .1*eye(4);
% Qf = Q;
% N3 = 121;
% options.integration_method = DirtranTrajectoryOptimization.MIDPOINT;
% prog3 = RobustDirtranTrajectoryOptimization(p,N3,D,E0,Q,R,Qf,duration,options);
% prog3 = prog3.addRobustCost(Q,5*R,Qf);
% %prog3 = prog3.addRobustInputConstraint();
% prog3 = prog3.addRobustStateConstraint(collision_constraint,2:(N3-1),1:2);
% prog3 = prog3.addStateConstraint(ConstantConstraint(double(x0)),1);
% prog3 = prog3.addInputConstraint(ConstantConstraint(u0),1);
% prog3 = prog3.addStateConstraint(ConstantConstraint(double(xf)),N3);
% prog3 = prog3.addInputConstraint(ConstantConstraint(u0),N3-1);
% %prog3 = prog3.addStateConstraint(BoundingBoxConstraint([0], [inf]), 2:(N-1), 3);
% prog3 = prog3.addStateConstraint(collision_constraint,2:N3-1,1:2);
% prog3 = prog3.addRunningCost(@cost);
% prog3 = prog3.setSolverOptions('snopt','majoroptimalitytolerance',1e-3);
% prog3 = prog3.setSolverOptions('snopt','minoroptimalitytolerance',1e-4);
% prog3 = prog3.setSolverOptions('snopt','majorfeasibilitytolerance',1e-3);
% prog3 = prog3.setSolverOptions('snopt','minorfeasibilitytolerance',1e-4);
% prog3 = prog3.setSolverOptions('snopt','superbasicslimit',5000);
% prog3 = prog3.setSolverOptions('snopt','iterationslimit',100000);
% 
% prog3 = addPlanVisualizer(p,prog3);
% tic
% [xtraj3,utraj3,z3,F3,info3] = prog3.solveTraj(tf0,traj_init);
% toc
% 
% v.playback(xtraj2, struct('slider',true));
% 
% tsamp3 = linspace(xtraj2.tspan(1), xtraj2.tspan(2), N);
% xsamp3 = xtraj2.eval(tsamp2);
% usamp3 = utraj2.eval(tsamp2);
% 
% figure();
% subplot(3,1,1);
% plot(tsamp1, xsamp1(1,:));
% hold on;
% plot(tsamp2, xsamp2(1,:));
% plot(tsamp3, xsamp3(1,:));
% subplot(3,1,2);
% plot(tsamp1, xsamp1(2,:));
% hold on;
% plot(tsamp2, xsamp2(2,:));
% plot(tsamp3, xsamp3(2,:));
% subplot(3,1,3);
% plot(tsamp1, xsamp1(3,:));
% hold on;
% plot(tsamp2, xsamp2(3,:));
% plot(tsamp3, xsamp3(3,:));
% 
% figure();
% subplot(4,1,1);
% plot(tsamp1, usamp1(1,:));
% hold on;
% plot(tsamp2, usamp2(1,:));
% plot(tsamp3, usamp3(1,:));
% subplot(4,1,2);
% plot(tsamp1, usamp1(2,:));
% hold on;
% plot(tsamp2, usamp2(2,:));
% plot(tsamp3, usamp3(2,:));
% subplot(4,1,3);
% plot(tsamp1, usamp1(3,:));
% hold on;
% plot(tsamp2, usamp2(3,:));
% plot(tsamp3, usamp3(3,:));
% subplot(4,1,4);
% plot(tsamp1, usamp1(4,:));
% hold on;
% plot(tsamp2, usamp2(4,:));
% plot(tsamp3, usamp3(4,:));

end



