function [psi, fNorm] = poincareMap(N)

p = SoftPaddleHybrid();
plantSim = SimulinkModel(p.getModel());
% take the frames from the simulink model and use those for the simulation of plant and controller
p = setOutputFrame(p, getOutputFrame(plantSim));
p = setInputFrame(p, getInputFrame(plantSim));

c = SoftPaddlePositionController(p);

x0 = p.getInitialState();

f = @(psi) costFun1Step(psi,x0,plantSim,c);

psi = linspace(-0.1, 0.1, N);
fNorm = linspace(-0.1, 0.1, N);

for i = 1:length(psi)
    fNorm(i) = norm(f(psi(i)))^2;
    fprintf(['psi = ', num2str(psi(i)), ', ||f||^2 = ', num2str(fNorm(i)), '\n'])
end

xi = x0([3:4, 7:8]+1);


figure(30), clf
plot(psi, fNorm, 'LineWidth', 2)
xlabel('$\psi$', 'Interpreter', 'LaTeX')
ylabel('$|f|^2$', 'Interpreter', 'LaTeX')
title(['x0 = ', num2str(round(xi,3)')])

end