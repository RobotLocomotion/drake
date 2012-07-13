function ppmathtest

a = PPTrajectory(zoh([0 1 2 3],randn(2,3,4)));
b = a';

for t=.5:1:2.5
  valuecheck(a.eval(t),b.eval(t)');
end


b = PPTrajectory(zoh([0 1 2 3],randn(3,2,4)));
c=a*b;

for t=.5:1:2.5
  valuecheck(a.eval(t)*b.eval(t),c.eval(t));
end
