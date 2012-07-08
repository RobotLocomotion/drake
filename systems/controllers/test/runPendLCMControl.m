
load drake_config;
p = path;
addpath([conf.root,'/examples/Pendulum']);

runLCM(PendulumEnergyControl(PendulumPlant));

% NOTEST
