checkDependency('yalmip');
expr = {};
ths = linspace(0, 2*pi, 17);
ths = ths(1:end-1);

cs = sdpvar(2,1);
theta = sdpvar(1,1);

constraints = [-1 <= cs <= 1];

for th = ths
  ai = rotmat(th) * [0;1];
  bi = 1;
  expr{end+1} = ai' * cs == bi;
end

figure(1)
clf
plot(polycone(cs, 1, 16));
title('polycone')
axis equal

figure(2)
clf
plot([constraints, hull(expr{:})]);
axis equal
title('hull');

figure(3)
clf
hold on
[c, sin_sector, cos_sector] = sinCosPiecewiseLinear(cs(2), cs(1), theta, -pi, pi);
plot(c, cs);
plot([c, binary(sin_sector), binary(cos_sector)], cs);
th = linspace(0, 2*pi);
plot(cos(th), sin(th), 'k-')
plot(pi/4, pi/4, 'ko') 
axis equal
title('humanoids14')

figure(4)
clf
hold on
[c, sector] = sinCosUnitCircleLinearEquality(cs(2), cs(1), theta, -pi, pi, 8);
plot(c, cs);
plot([c, binary(sector)], cs);
th = linspace(0, 2*pi);
plot(cos(th), sin(th), 'k-')
axis equal
title('circle equality')

