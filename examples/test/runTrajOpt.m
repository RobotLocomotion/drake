
%%
[p,xtraj1,utraj1,ltraj1,ljltraj1,z1,F1,info1,traj_opt1] = trajOpt();
v = p.constructVisualizer; v.playback(xtraj1,struct('slider',true));
display('iter 1')
%%
[p,xtraj2,utraj2,ltraj2,ljltraj2,z2,F2,info2,traj_opt2] = trajOpt(xtraj1,utraj1,ltraj1,ljltraj1,.1);
v = p.constructVisualizer; v.playback(xtraj2,struct('slider',true));
display('iter 2')
%%
[p,xtraj3,utraj3,ltraj3,ljltraj3,z3,F3,info3,traj_opt3] = trajOpt(xtraj2,utraj2,ltraj2,ljltraj2,.01);
v = p.constructVisualizer; v.playback(xtraj3,struct('slider',true));
display('iter 3')
%%
[p,xtraj4,utraj4,ltraj4,ljltraj4,z4,F4,info4,traj_opt4] = trajOpt(xtraj3,utraj3,ltraj3,ljltraj3,1e-3);
v = p.constructVisualizer; v.playback(xtraj4,struct('slider',true));
display('iter 4')
%%
[p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = trajOpt(xtraj4,utraj4,ltraj4,ljltraj4,0);
v = p.constructVisualizer; v.playback(xtraj,struct('slider',true));
display('iter 5')
%%

keyboard;
save('data/hopper_traj.mat','xtraj','utraj','ltraj','ljltraj','z','F');
