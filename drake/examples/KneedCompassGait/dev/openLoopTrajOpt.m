% N_list = 5:20;
N_list = 5:30;
T = .2;
for i=1:length(N_list)
  N = N_list(i);
  [p,v,xtraj,utraj,ltraj,z,F,info,traj_opt] = openLoopTrajOptTest(N,T);
  
  cost(i) = F;
  info_list(i) = info;
  h(i) = xtraj.tspan(2)/(N-1);
  display(sprintf('info: %d, N: %d, F: %e, C: %e',info,N,F,F/h(i)^3));
end