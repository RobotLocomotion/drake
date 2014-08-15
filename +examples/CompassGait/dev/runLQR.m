function runLQR

sys = SimulinkModel('cgLQRTest');

for i=1:5
  simulate(sys,[0 5],[0;0;0;0]+.01*randn(4,1));
end