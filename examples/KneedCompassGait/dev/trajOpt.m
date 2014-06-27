
%%
[p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = testNewTrajOpt();
v = p.constructVisualizer; v.playback(xtraj);
display('iter 1')
%%
[p,xtraj2,utraj2,ltraj2,ljltraj2,z2,F2,info2,traj_opt2] = testNewTrajOpt(xtraj,utraj,ltraj,ljltraj,.1);
v = p.constructVisualizer; v.playback(xtraj2);
display('iter 2')
%%
[p,xtraj3,utraj3,ltraj3,ljltraj3,z3,F3,info3,traj_opt3] = testNewTrajOpt(xtraj2,utraj2,ltraj2,ljltraj2,.01);
v = p.constructVisualizer; v.playback(xtraj3);
display('iter 3')
%%
[p,xtraj4,utraj4,ltraj4,ljltraj4,z4,F4,info4,traj_opt4] = testNewTrajOpt(xtraj3,utraj3,ltraj3,ljltraj3,1e-3);
v = p.constructVisualizer; v.playback(xtraj4);
display('iter 4')
%%
[p,xtraj5,utraj5,ltraj5,ljltraj5,z5,F5,info5,traj_opt5] = testNewTrajOpt(xtraj4,utraj4,ltraj4,ljltraj4,0);
v = p.constructVisualizer; v.playback(xtraj5);
display('iter 5')
%%
if info5 ~= 1
[p,xtraj5,utraj5,ltraj5,ljltraj5,z5,F5,info5,traj_opt5] = testNewTrajOpt(xtraj5,utraj5,ltraj5,ljltraj5,0);
v = p.constructVisualizer; v.playback(xtraj5);
end