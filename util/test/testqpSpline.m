function testqpSpline()

t0 = 0;
t1 = 1;
t2 = 2;
t3 = 3;

ts = [0, 1, 2, 3];
xs = [rand(6,1), rand(6,1), rand(6,1), rand(6,1)];
xd0 = rand(6,1);
xdf = rand(6,1);

options.optimize_knot_times = true;
coefs = qpSpline(ts, xs, xd0, xdf, options, true);