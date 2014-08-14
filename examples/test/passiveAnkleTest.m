%%
[p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = runHopperTrajOpt_passiveankle();
%%
[p2,xtraj2,utraj2,ltraj2,ljltraj2,z2,F2,info2,traj_opt2] = runHopperTrajOpt_passiveankle(xtraj,utraj,ltraj,ljltraj,.1);
%%
[p3,xtraj3,utraj3,ltraj3,ljltraj3,z3,F3,info3,traj_opt3] = runHopperTrajOpt_passiveankle(xtraj2,utraj2,ltraj2,ljltraj2,.01);