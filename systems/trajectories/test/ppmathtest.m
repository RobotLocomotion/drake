function ppmathtest

a = PPTrajectory(spline([0 1 2 3],randn(2,3,4)));
b = a';

for t=.5:1:2.5
  valuecheck(a.eval(t),b.eval(t)');
end


b = PPTrajectory(spline([0 1 1.5 2 3],randn(3,2,5)));
c=a*b;

for t=.25:.25:2.75
  valuecheck(a.eval(t)*b.eval(t),c.eval(t));
end

c = a' + b;

for t=.25:.25:2.75
  valuecheck(a.eval(t)'+b.eval(t),c.eval(t));
end
