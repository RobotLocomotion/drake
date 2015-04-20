function runLQR

pd = GliderPlant;
pv = GliderVisualizer(pd);

% obtain nomial trajectories via dircol with end constraints
try
  trajs=load('glider_trajs.mat');

  xtraj=trajs.xtraj;
  utraj=trajs.utraj;

  utraj = setOutputFrame(utraj,pd.getInputFrame);
  xtraj = setOutputFrame(xtraj,pd.getStateFrame);
catch
  [utraj,xtraj]=runDircolPerching(pd);
end
pd = pd.setInputLimits(-inf,inf);


figure()
plot(ppval(utraj.pp.breaks,utraj.pp)');

c = GliderLQR(pd,xtraj,utraj);

sys=cascade(feedback(pd,c),pv);

for i=1:1
  simulate(sys,xtraj.tspan,[-3.5; 0.2; 0; 0; 7+0.2*randn(1,1); 0;0]);
end

end
