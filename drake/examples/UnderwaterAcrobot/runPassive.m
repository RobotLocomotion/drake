function runPassive()
% Simulate the passive acrobot

p = UnderwaterAcrobotPlant;
v = UnderwaterAcrobotVisualizer(p);

xtraj = simulate(p,[0 10],randn(4,1));
v.playback(xtraj);

%playbackAVI(v,xtraj,'passive');
end
