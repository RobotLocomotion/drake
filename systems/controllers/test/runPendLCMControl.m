function runPendLCMControl

load drake_config;
oldpath = addpath([conf.root,'/examples/Pendulum']);

p=PendulumPlant;
p = setInputLimits(p,-inf,inf);
runLCM(PendulumEnergyShaping(p),[],struct('tspan',[0 2]));

path(oldpath);
