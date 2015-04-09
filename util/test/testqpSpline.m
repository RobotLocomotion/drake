function testqpSpline()

ndof = 6;
ts = [0, 1, 2, 3];
xs = [rand(ndof,1), rand(ndof,1), rand(ndof,1), rand(ndof,1)];
xd0 = rand(ndof,1);
xdf = rand(ndof,1);

options.optimize_knot_times = true;
coefs = qpSpline(ts, xs, xd0, xdf, options, false);

options.optimize_knot_times = false;
options.use_mex = false;

[coefs, objval] = qpSpline(ts, xs, xd0, xdf, options, false);
t = timeit(@() qpSpline(ts, xs, xd0, xdf, options, false), 2);

options.use_mex = true;
[coefs_mex, objval_mex] = qpSpline(ts, xs, xd0, xdf, options, false);
t_mex = timeit(@() qpSpline(ts, xs, xd0, xdf, options, false), 2);

valuecheck(coefs_mex, coefs);
valuecheck(objval_mex, objval);

fprintf('t: %0.5f, t_mex: %0.5f\n', t, t_mex);

end
