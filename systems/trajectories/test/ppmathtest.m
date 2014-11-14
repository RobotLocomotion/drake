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

c = PPTrajectory(spline([0 1 2 3],randn(2,3,4)));
d = [a;c];
e = a(1:2,:);

for t=.25:.25:2.75
  valuecheck(a.eval(t),e.eval(t));
end

c(1,2) = ConstantTrajectory(2);
for t=.25:.25:2.75
  tmp = c.eval(t);
  valuecheck(tmp(1,2),2);
end

c = ConstantTrajectory(randn(1,3));
d = [c;a];

for t=.25:.25:2.75
  valuecheck(d.eval(t),[c.pt;a.eval(t)]);
end


apts(:,:,1) = 2*eye(3);
apts(:,:,2) = 4*eye(3);
apts(:,:,3) = 6*eye(3);
apts(:,:,4) = apts(:,:,3);
a = PPTrajectory(zoh([0 1 2 3],apts));
b = inv(a);
typecheck(b,'PPTrajectory');

valuecheck(eval(b,.5),.5*eye(3));
valuecheck(eval(b,1.5),.25*eye(3));
valuecheck(eval(b,2.5),eye(3)/6);

fr1 = CoordinateFrame('test',2,'x');
fr2 = CoordinateFrame('test-x0',2,'x');
x0=[5;5];
fr1.addTransform(AffineTransform(fr1,fr2,eye(2),-x0));
fr2.addTransform(AffineTransform(fr2,fr1,eye(2),x0));

V = QuadraticLyapunovFunction(fr1,[4,0; 2,3],[1;1],.5);
valuecheck(getLevelSetVolume(V),getLevelSetVolume(V.inFrame(fr2)));

% Test time scaling
scale = rand(1) + 1.01;
a = PPTrajectory(spline([0 1 2 3], randn(3,4)));
b = a.scaleTime(scale);
for t = .25:.25:2.75
  valuecheck(a.eval(t), b.eval(scale*t));
end

scale = rand(1) + 1.01;
a = PPTrajectory(spline([0 1 2,3], randn(1,4)));
b = a.scaleTime(scale);
for t= .25:.25:2.75
  valuecheck(a.eval(t), b.eval(scale*t));
end

% Test time scaling with concatenated trajectories
a = PPTrajectory(spline([0 1 2 3], randn(1,4)));
b = PPTrajectory(spline([0 1 2 3], randn(1,4)));
c = a.vertcat(b);
scale = rand(1) + 1.01;
d = c.scaleTime(scale);
for t= .25:.25:2.75
  valuecheck(c.eval(t), d.eval(scale*t));
end

% Test vertical concatenation maintaining dimension of the trajectory
a = PPTrajectory(spline(1:5, randn(4,5)));
b = PPTrajectory(spline(1:5, randn(4,5)));
c = a.vertcat(b);

sizecheck(c.eval([1.5, 2.5]), [8, 2]);