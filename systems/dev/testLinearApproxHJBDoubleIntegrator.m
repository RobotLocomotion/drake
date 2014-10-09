function testLinearApproxHJBDoubleIntegrator

tmp = addpathTemporary(fullfile(getDrakePath(),'examples'));

p = DoubleIntegrator;
gxfun = @(x) .5*x'*x;  % aka Q = eye(2)
R = 1;
xbins = {[-3:.2:3],[-4:.2:4]};
linearApproxHJB(p,gxfun,R,xbins);
