function pprefinetest

a = PPTrajectory(spline([0 1 2 3],randn(2,4)));

tr = 0:.25:3;
b = refine(a,tr);
bpp = pprfn(a.pp,tr);

ts = 0:.05:3;
valuecheck(b.eval(ts),ppval(bpp,ts));
