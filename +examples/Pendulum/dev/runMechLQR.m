function runMechLQR

oldpath = addpath(fullfile(pwd,'..'));

p = SimulinkModel('mech');
v = PendulumVisualizer();
p = setTIFlag(p,true);
c = tilqr(p,[pi;0],0,diag([10,1]),1);

sys = feedback(p,c);
%sys = setSimulinkParam(sys,'VisDuringSimulation','on');
sys = cascade(sys,v);
sys = setSimulinkParam(sys,'VisDuringSimulation','off');

for i=1:5
  simulate(sys,[0 4],[pi;0]+0.05*randn(2,1));
end

path(oldpath);