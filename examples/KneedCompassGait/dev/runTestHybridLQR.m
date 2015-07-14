Q = 100*eye(12);
Qf = 100*eye(12);
R = eye(3);

lqr_options.periodic = true;


R_periodic = zeros(12);
R_periodic(1,1) = 1; %x
R_periodic(2,2) = 1; %z
R_periodic(3,3) = 1; %pitch-hip w/symmetry
R_periodic(3,5) = 1; %pitch-hip w/symmetry
R_periodic(4,6) = 1; %knee w/symmetry
R_periodic(6,4) = 1; %knee w/symmetry
R_periodic(5,5) = -1; %hip w/symmetry

R_periodic(7,7) = 1; %x-vel
R_periodic(8,8) = 1; %z-vel
R_periodic(9,9) = 1; %pitch-hip w/symmetry
R_periodic(9,11) = 1; %pitch-hip w/symmetry
R_periodic(10,12) = 1; %knee w/symmetry
R_periodic(12,10) = 1; %knee w/symmetry
R_periodic(11,11) = -1; %hip w/symmetry

lqr_options.periodic_jump = R_periodic;


[c,Ktraj,Straj,Ptraj,Btraj,Straj_full,Ftraj,xtraj,utraj] = test_hybridlqr(constrained_plants,xtraj,utraj,Q,R,Qf,lqr_options);


%%
c{1} = c{1}.setOutputFrame(r.getInputFrame);
c{1} = c{1}.setInputFrame(r.getOutputFrame);
c{2} = c{2}.setOutputFrame(r.getInputFrame);
c{2} = c{2}.setInputFrame(r.getOutputFrame);

sys_cl1 = r.feedback(c{1});
sys_cl2 = r.feedback(c{2});

x0 = xtraj{1}.eval(0);

%%
traj_cl1 = sys_cl1.simulate(xtraj{1}.tspan,x0);
v.playback(traj_cl1);
tspan2 = xtraj{2}.tspan;
tspan2(1) = floor(tspan2(1)/r.timestep)*r.timestep;
tspan2(2) = tspan2(1) + floor(diff(tspan2)/r.timestep)*r.timestep;
x0 = traj_cl1.eval(traj_cl1.tspan(2));
traj_cl2 = sys_cl2.simulate(tspan2,x0);
v.playback(traj_cl2);
x0 = R_periodic*traj_cl2.eval(tspan2(2)); x0(1) = 0;
v.drawWrapper(0,traj_cl2.eval(tspan2(2)))