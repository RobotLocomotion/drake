function aniROATest

oldpath = addpath('..');

robot = AcrobotPlant;
robot = robot.setInputLimits(-inf,inf);
[c,V] = AcrobotLQR(robot);
sys = feedback(robot,c);
x0 = [pi;0;0;0];
pp = sys.taylorApprox(0,x0,0,3);

options=struct();
options.degL1=6;
%options.method='bilinear';

V = regionOfAttraction(pp,V,options);
%V2 = regionOfAttraction(pp,x0,10*(pp.p_x-x0)'*c.S*(pp.p_x-x0),options);
%V3 = regionOfAttraction(pp,x0,V2,options);

hold on;
plotFunnel(V,[],[2 4]);
%plotFunnel(V2,[],[2 4]);

legend('without guess','with guess');

path(oldpath);
