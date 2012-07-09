
load drake_config;
oldpath = addpath([conf.root,'/examples/Pendulum']);

p=PendulumPlant;
p = setInputLimits(p,-inf,inf);
runLCM(PendulumEnergyControl(p),[],struct('tspan',[0 2]));

path(oldpath);
