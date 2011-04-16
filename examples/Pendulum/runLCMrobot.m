function runLCMrobot

% NOTEST

p=PendulumPlant();
p=setInputLimits(p,-inf,inf);

v=PendulumVisualizer();
v.draw(0,zeros(2,1)); drawnow;

sys = cascade(p,sampledData(v,.025),true);

runLCM(sys,PendulumLCMCoder,'u','xhat');
