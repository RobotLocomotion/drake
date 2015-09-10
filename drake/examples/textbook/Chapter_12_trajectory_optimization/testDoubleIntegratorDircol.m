function testDoubleIntegratorDircol

% note: requires Drake ver >= 0.9.7

cd(fullfile(getDrakePath,'examples'));
DoubleIntegrator.runDircol;

% make sure you take a look at the code!
edit('DoubleIntegrator.runDircol')

