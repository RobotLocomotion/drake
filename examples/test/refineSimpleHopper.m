clear all
load data/simplehopper_traj
t = xtraj.pp.breaks;
tr0 = t(end-7);
trf = t(end-4);

x0 = xtraj.eval(tr0);
xf = xtraj.eval(trf);

xtraj_shift = xtraj.shiftTime(-tr0);
utraj_shift = utraj.shiftTime(-tr0);
ltraj_shift = ltraj.shiftTime(-tr0);

[p,xtraj,utraj,ltraj,ljltraj,z,F,info,traj_opt] = refineSimpleTrajOpt(x0,xf,trf-tr0,xtraj,utraj,ltraj,ljltraj,0.1);

t = xtraj.pp.breaks;
x = xtraj.eval(t)
l = ltraj.eval(t);