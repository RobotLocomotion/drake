function [x, xplus] = poincareMap_x(N)

p = SoftPaddleHybrid();
plantSim = SimulinkModel(p.getModel());
% take the frames from the simulink model and use those for the simulation of plant and controller
p = setOutputFrame(p, getOutputFrame(plantSim));
p = setInputFrame(p, getInputFrame(plantSim));

c = SoftPaddlePositionController(p);

x0 = p.getInitialState();
x0(5) = 5;

x = linspace(-0.039375, -0.039328, N);
xplus = zeros(N,1);

for i = 1:length(x)
    x0(4) = x(i);
    f = @(psi) costFun1Step_x(psi,x0,plantSim,c);
    xplus(i) = f(0);
    fprintf(['x = ', num2str(x(i)), ', x+ = ', num2str(xplus(i)), '\n'])
end

xi = x0([3:4, 7:8]+1);


figure(30), clf
plot(x, xplus, 'LineWidth', 2)
xlabel('$x_0$', 'Interpreter', 'LaTeX')
ylabel('$x^+$', 'Interpreter', 'LaTeX')
title(['x0 = ', num2str(round(xi,3)')])

end