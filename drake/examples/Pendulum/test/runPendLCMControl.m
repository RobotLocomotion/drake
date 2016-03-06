function runPendLCMControl

tmpdir = addpathTemporary([getDrakePath,'/examples/Pendulum']);

p=PendulumPlant;
p = setInputLimits(p,-inf,inf);
runLCM(PendulumEnergyShaping(p),[],struct('tspan',[0 2]));

