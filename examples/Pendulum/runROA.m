function runROA

p = PendulumPlant;
%c = PendulumLQR(pd);
%sys = feedback(pd,c);

phasePortrait(p);
hold on;

x0 = zeros(2,1);
pp = p.taylorApprox(0,x0,0,3);  % make polynomial approximation

V=regionOfAttraction(pp,zeros(2,1));
xroa=getLevelSet(V);
plot(xroa(1,:)+x0(1),xroa(2,:)+x0(2),'r');





