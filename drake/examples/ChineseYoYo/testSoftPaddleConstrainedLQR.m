%[p,v,xtraj,utraj,z,ltraj,F,info,traj_opt] = testConstrainedDircol(10);

%Loading in the recorded trajectory
load('trajectorytoStabilizeShort.mat');
%xtraj = xtraj.traj{2};
%utraj = ytraj(10);%ytraj.traj{2}(10);
%tt=getBreaks(xtraj);

%build up hybrid plant
p = SoftPaddleHybrid();
utraj = utraj.setOutputFrame(getInputFrame(p));
xtraj = xtraj.setOutputFrame(getOutputFrame(p));


v = p.constructVisualizer();

x0 = eval(xtraj,tt(1));
v.drawWrapper(0,x0);
v.playback(xtraj,struct('slider',true));

%%Change to just in_contact plant
plant = p.in_contact;
%adjust trajectories for 8 states
xtrajIC = xtraj(2:9);
xtrajIC = setOutputFrame(xtrajIC, getOutputFrame(plant));
utrajIC = utraj; %unneccessary
utrajIC = setOutputFrame(utrajIC, getInputFrame(plant)); %unneccessary

% run lqr
xdim = p.in_contact.num_positions+ p.in_contact.num_velocities;
qw = [0,0,1,1];
Q = 10*diag([qw,qw]);
R = 1;
options.alpha_1 = 100; % regularization term. Larger means more accurate but stiffer ODE to integrate.
[c,Ktraj,Straj,Ptraj,Btraj,Ftraj,Straj_full] = constrainedtvlqr(plant,xtrajIC,utrajIC,Q,R,Q,options);

% build closed loop plant
sys_cl = plant.feedback(c);
%%

% get perturbed initial state
x0 = xtrajIC.eval(0) + 0.03*randn(xdim,1);
x0 = plant.resolveConstraints(x0);

traj_clIC = sys_cl.simulate(xtrajIC.tspan,x0);

%build the 9 state trajectory
traj_cl = [ConstantTrajectory(2);traj_clIC]; %set mode to be 2
traj_cl = traj_cl.setOutputFrame(p.getOutputFrame);
playback(v,traj_cl,struct('slider','true'))

t = linspace(xtrajIC.tspan(1),xtrajIC.tspan(2),1e3);

%%
figure(1),clf
plot(t,xtrajIC.eval(t),t,traj_clIC.eval(t),'--');
figure(2),clf
err = xtrajIC.eval(t)-traj_clIC.eval(t);
plot(t,err);
figure(3),clf
Snorm = diag(sqrt(err'*err));
plot(t,Snorm);
