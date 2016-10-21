function psid = NLPoincare(x0)

warning('off');

p = SoftPaddleHybrid();
plantSim = SimulinkModel(p.getModel());
% take the frames from the simulink model and use those for the simulation of plant and controller
p = setOutputFrame(p, getOutputFrame(plantSim));
p = setInputFrame(p, getInputFrame(plantSim));

c = SoftPaddlePositionController(p);

if nargin < 1
    x0 = Point(getStateFrame(p));
    x0.m = 1;
    x0.load_x = -0.5;
    x0.load_z = 4.5;
    x0 = double(x0);
    x0(2:end) = resolveConstraints(p.no_contact,x0(2:end));
end
    
psiGuess = 0;

fun = @(psi) myfun(psi,x0,plantSim,c);
options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', 'Display', 'iter');
% X = lsqnonlin(fun, psiGuess, -0.2, 0.2, options);
[psid,resnorm,residual,exitflag,output] = lsqnonlin(fun, psiGuess, [], [], options);

end