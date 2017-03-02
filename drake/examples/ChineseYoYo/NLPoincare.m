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
    
% psiGuess = 0;
% fun = @(psi) costFun1Step(psi,x0,plantSim,c);

psiGuess = [-0.1; -0.05; 0; 0; 0]*0;
% psiGuess = [-0.0266   -0.0103    0.0096    0.0045   -0.0009]';      % for a start load_x = -0.5, load_z = 4.5
% psiGuess = [-0.0361   -0.0126    0.0155    0.0044   -0.0038]';      % for a start load_x = -0.75, load_z = 4.5
% psiGuess = [-0.0369   -0.0119    0.0157    0.0039   -0.0035]';      % for a start load_x = -0.75, load_z = 4.75
% psiGuess = [-0.05   -0.01    0    0   0]';      % for a start load_x = -0.75, load_z = 4.75

% psiGuess = [-0.0714; 0.0279; 0.0361; -0.007; -0.0173];

% psiGuess = [-0.0714; 0.0279; 0.0361; -0.007; -0.0173; 5];
% psiGuess = [zeros(5,1); 5];

fun = @(psi) costFun(psi,x0,plantSim,c);
options = optimoptions('lsqnonlin', 'Algorithm', 'levenberg-marquardt', 'Display', 'iter');
% X = lsqnonlin(fun, psiGuess, -0.2, 0.2, options);
[psid,resnorm,residual,exitflag,output] = lsqnonlin(fun, psiGuess, [], [], options);

end