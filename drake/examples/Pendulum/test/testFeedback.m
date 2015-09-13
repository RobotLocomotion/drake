function testFeedback

tmp = addpathTemporary([getDrakePath,'/examples/Pendulum']);

p = PendulumPlant();
c = balanceLQR(p);
  
sys1=feedback(p,c);
typecheck(sys1,'DrakeSystem');
  
p2 = SimulinkModel(p.getModel());
p2 = setInputFrame(p2,p.getInputFrame());
p2 = setStateFrame(p2,p.getStateFrame());
p2 = setOutputFrame(p2,p.getOutputFrame());

sys2=feedback(p2,c);
typecheck(sys2,'SimulinkModel');

valuecheck(dynamics(p,0,[pi;0],0),zeros(2,1));
valuecheck(output(c,0,[],zeros(2,1)),0);
c2 = c.inInputFrame(p.getOutputFrame);
valuecheck(output(c2,0,[],[pi;0]),0);
valuecheck(output(SimulinkModel(c.getModel()),0,[],zeros(2,1)),0);
valuecheck(output(SimulinkModel(c2.getModel()),0,[],[pi;0]),0);
valuecheck(dynamics(p2,0,[pi;0],0),zeros(2,1));
valuecheck(dynamics(sys1,0,[pi;0],0),zeros(2,1));
valuecheck(dynamics(sys2,0,[pi;0],0),zeros(2,1));

for i=1:40
  t=rand; x=randn(2,1); u=randn;
  valuecheck(output(sys1,t,x,u),output(sys2,t,x,u));
  valuecheck(dynamics(sys1,t,x,u),dynamics(sys2,t,x,u));
end

% polynomial feedback
p = setInputLimits(p,-inf,inf);  % todo: another case where I get rid of this
p = taylorApprox(p,0,Point(p.getStateFrame,[pi;0]),[],3);
sys1=feedback(p,c);
%  typecheck(sys1,'PolynomialSystem');  % not a polynomial system anymore
%  due to wrapping coordinate transform

p2 = SimulinkModel(p.getModel());
p2 = setInputFrame(p2,p.getInputFrame());
p2 = setStateFrame(p2,p.getStateFrame());
p2 = setOutputFrame(p2,p.getOutputFrame());
sys2=feedback(p2,c);
typecheck(sys2,'SimulinkModel');

for i=1:40
  t=rand; x=randn(2,1); u=randn;
  valuecheck(output(sys1,t,x,u),output(sys2,t,x,u));
  valuecheck(dynamics(sys1,t,x,u),dynamics(sys2,t,x,u));
end

