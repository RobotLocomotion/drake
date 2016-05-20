function linear_gaussian_dt

% simple test to implement the same check as in LinearGaussianDT, but using
% the DrakeSystemWGaussianNoise version

sys = LinearSystem([],[],0,[],1,[]);  % xdn = w; y=x;
ssys = DrakeSystemWGaussianNoise(sys,[],2,0);

xtraj = simulate(ssys,[0 1000]);

fnplt(xtraj);

% check that I'm actually getting the variance i expect
valuecheck(2,var(eval(xtraj,getBreaks(xtraj))),.2);

% note that this test is currently known to fail with small probability
% https://github.com/RobotLocomotion/drake/issues/366
