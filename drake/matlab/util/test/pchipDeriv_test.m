function pchipDeriv_test

t = 0:.5:5;
y = [sin(t); 3*sin(t)];
dy = [cos(t); 3*cos(t)];

pp = pchipDeriv(t,y,dy);

ts = 0:0.05:5;
plot(ts,ppval(pp,ts),'r');