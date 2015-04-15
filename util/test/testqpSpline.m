function testqpSpline()
testMex();
% testTiming(); doesn't assert anything and timeit isn't available everywhere
testObjectiveValue();
end

function testMex()
ndof = 6;
ts = [0, 1, 2, 3];
xs = [rand(ndof,1), rand(ndof,1), rand(ndof,1), rand(ndof,1)];
xd0 = rand(ndof,1);
xdf = rand(ndof,1);

options.optimize_knot_times = false;
options.use_mex = false;
[coefs, ts, objval] = qpSpline(ts, xs, xd0, xdf, options);

options.use_mex = true;
[coefs_mex, ts_mex, objval_mex] = qpSpline(ts, xs, xd0, xdf, options);

valuecheck(coefs_mex, coefs);
valuecheck(ts_mex, ts);
valuecheck(objval_mex, objval);
end

function testTiming()
ndof = 6;
ts = [0, 1, 2, 3];
xs = [rand(ndof,1), rand(ndof,1), rand(ndof,1), rand(ndof,1)];
xd0 = rand(ndof,1);
xdf = rand(ndof,1);

options.optimize_knot_times = false;

options.use_mex = false;
t = timeit(@() qpSpline(ts, xs, xd0, xdf, options), 2);

options.use_mex = true;
t_mex = timeit(@() qpSpline(ts, xs, xd0, xdf, options), 2);

fprintf('t: %0.5f, t_mex: %0.5f\n', t, t_mex);
% assert(t_mex < t);
end

function testObjectiveValue()
ndof = 6;
ts = [0, 1, 2, 3];
xs = [rand(ndof,1), rand(ndof,1), rand(ndof,1), rand(ndof,1)];
xd0 = rand(ndof,1);
xdf = rand(ndof,1);

options.use_mex = true;
[coefs, ts, objval] = qpSpline(ts, xs, xd0, xdf, options);

pp = mkpp(ts, coefs, 6);

tt = linspace(ts(1), ts(end), 1000);

atraj = fnder(pp, 2);
as = ppval(atraj, tt);
ns = sum(as.^2,1);
relative_error = abs(sum(ns) * (tt(2) - tt(1)) - objval) / objval;
rangecheck(relative_error, 0, 1e-2);

show_plots = false;
if show_plots
  ps = ppval(pp, tt);
  figure(15);
  clf
  for j = 1:6
    subplot(6, 1, j)
    hold on
    plot(tt, ps(j,:));
    plot(ts, xs(j,:), 'ro');
  end
  
  figure(16);
  clf
  vs = ppval(fnder(pp, 1), tt);
  for j = 1:6
    subplot(6, 1, j)
    plot(tt, vs(j,:));
  end
  
  figure(17);
  clf
  for j = 1:6
    subplot(6,1,j);
    plot(tt, as(j,:));
  end
end
end
