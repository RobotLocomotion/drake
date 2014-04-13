function runLQR()
%% run the lqr controller from a handful of initial conditions

disp('Initializing Plant');
p = UnderwaterAcrobotPlant;
v = UnderwaterAcrobotVisualizer(p);

disp('Generating Controller');
c = balanceLQR(p);
sys = feedback(p,c);

for i=1:5
    disp(['Simulation ' mat2str(i,1)]);
  xtraj=simulate(sys,[0 5],double(p.xG)+0.02*randn(4,1));
  v.playback(xtraj);
end

%playbackAVI(v,xtraj,'lqr');
end
