function testCubicSplineCoefficients()

pchip_construction_time = 0;
pchip_eval_time = 0;
csc_construction_time = 0;
csc_eval_time = 0;
for i = 1:10
  dim = 2*i;
  ts = [0, rand()+0.5];
  qs = rand(dim, 2);
  qdots = rand(dim, 2);

  t0 = tic();
  pp = pchipDeriv(ts, qs, qdots);
  [~, coefs, l, k, d] = unmkpp(pp);
  pchip_construction_time = pchip_construction_time + toc(t0);

  coefs = reshape(coefs, [d, l, k]);

  t0 = tic();
  coefs_csc = cubicSplineCoefficients(ts(2), qs(:,1), qs(:,2), qdots(:,1), qdots(:,2));
  csc_construction_time = csc_construction_time + toc(t0);

  % verify that the coefficients are the same
  valuecheck(coefs, coefs_csc);

  % verify that the splines evaluate to the same values
  for j = 1:10
    t = rand() * ts(2);
    t0 = tic();
    v1 = ppval(pp, t);
    pchip_eval_time = pchip_eval_time + toc(t0);

    t0 = tic();
    v2 = evalCubicSplineSegment(t, coefs_csc);
    csc_eval_time = csc_eval_time + toc(t0);

    valuecheck(v1, v2);
  end
end

fprintf(1, 'pchip/cubicSplineCoefficients time ratio: construction: %f eval: %f\n', pchip_construction_time / csc_construction_time, pchip_eval_time / csc_eval_time);

