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


fr1 = CoordinateFrame('test',2,'x');
fr2 = CoordinateFrame('test-x0',2,'x');
x0=[5;5];
fr1.addTransform(AffineTransform(fr1,fr2,eye(2),-x0));
fr2.addTransform(AffineTransform(fr2,fr1,eye(2),x0));

V = QuadraticLyapunovFunction(fr1,[4,0; 2,3],[1;1],.5);
valuecheck(getLevelSetVolume(V),getLevelSetVolume(V.inFrame(fr2)));

