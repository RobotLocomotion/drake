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

V = regionOfAttraction(pp,x0,V,options);
%V2 = regionOfAttraction(pp,x0,10*(pp.p_x-x0)'*c.S*(pp.p_x-x0),options);
%V3 = regionOfAttraction(pp,x0,V2,options);

xroa = getLevelSet(subs(V,pp.p_x([1 3]),x0([1 3])),x0([2 4]));
%xroa2 = getLevelSet(subs(V2,pp.p_x([1 3]),x0([1 3])),x0([2 4]));

hold on
plot(xroa(1,:),xroa(2,:),'r');
%plot(xroa2(1,:),xroa2(2,:),'b');

legend('without guess','with guess');

path(oldpath);
