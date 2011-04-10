function aniROATest

robot = AcrobotPlant;
robot = robot.setInputLimits(-inf,inf);
c = AcrobotLQR(robot);
sys = feedback(robot,c);
x0 = c.x0;
pp = sys.taylorApprox(0,x0,[],3);

options=struct();
options.monom_order=1;
options.method='pablo';

V = regionOfAttraction(pp,[],[],options);
V2 = regionOfAttraction(pp,[],pp.p_x'*c.S*pp.p_x,options);

xroa = getLevelSet(subs(V,pp.p_x([1 3]),zeros(2,1)));
xroa2 = getLevelSet(subs(V2,pp.p_x([1 3]),zeros(2,1)));

hold on
plot(xroa(1,:)+x0(1),xroa(2,:)+x0(2),'r');
plot(xroa2(1,:)+x0(1),xroa2(2,:)+x0(2),'b');

legend('without guess','with guess');


