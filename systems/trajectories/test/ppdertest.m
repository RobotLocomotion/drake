function ppdertest

a = PPTrajectory(spline([0 1 2 3],randn(2,4)));

for o=1:4
  b = fnder(a,o);
  bpp = fnder(a.pp,o);

  ts = 0:.1:3;
  valuecheck(b.eval(ts),ppval(bpp,ts));
end
