cmd "PendulumVisualizer" {
    exec = "matlab -nosplash -nodesktop -r \"cd([getDrakePath(),'/examples/Pendulum']); v=PendulumVisualizer(); runLCM(v);\"";
    host = "localhost";
}
cmd "PendulumPlant" {
    exec = "matlab -nosplash -nodesktop -r \"cd([getDrakePath(),'/examples/Pendulum']); p=PendulumPlant(); runLCM(p);\" ";
    host = "localhost";
}
cmd "PendulumEnergyControl" {
    exec = "matlab -nosplash -nodesktop -r \"cd([getDrakePath(),'/examples/Pendulum']); c=PendulumEnergyControl(setInputLimits(PendulumPlant,-inf,inf)); runLCM(c);\"";
    host = "localhost";
}

